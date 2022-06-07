use crseo::pssn::TelescopeError;
use crseo::{Diffractive, FromBuilder, Gmt, PSSn, ShackHartmann, Source};
use dos_actors::{
    clients::{
        arrow_client::{Arrow, Get},
        ceo,
        gmt_state::GmtState,
    },
    prelude::*,
    Update,
};
use linya::{Bar, Progress};
use skyangle::Conversion;
use std::{sync::Arc, time::Duration};
use tokio::sync::Mutex;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();
    let sim_sampling_frequency = 1_000_usize;
    const CFD_DELAY: usize = 10; // seconds
    const EXPOSURE_RATE: usize = 30_000;
    let sim_duration = (EXPOSURE_RATE / sim_sampling_frequency) as f64;
    log::info!("Simulation duration: {:6.3}s", sim_duration);

    let gmt_builder = Gmt::builder().m1_n_mode(162);
    let bench = ceo::OpticalModel::builder()
        .gmt(gmt_builder)
        .source(Source::builder().zenith_azimuth(vec![6f32.from_arcmin()], vec![0f32]))
        .options(vec![
            ceo::OpticalModelOptions::ShackHartmann {
                options: ceo::ShackHartmannOptions::Diffractive(
                    ShackHartmann::<Diffractive>::builder(),
                ),
                flux_threshold: 0.,
            },
            /*ceo::OpticalModelOptions::Atmosphere {
                                    builder: {
                let atm_duration = 20f32;
                let atm_n_duration = Some((sim_duration / atm_duration as f64).ceil() as i32);
                let atm_sampling = 48 * 16 + 1;
            ATMOSPHERE::new().ray_tracing(
                            25.5,
                            atm_sampling,
                            20f32.from_arcmin(),
                            atm_duration,
                            Some("/fsx/atmosphere/atm_15mn.bin".to_owned()),
                            atm_n_duration,
                        )},
                                    time_step: (sim_sampling_frequency as f64).recip(),
                                },*/
            ceo::OpticalModelOptions::PSSn(ceo::PSSnOptions::Telescope(
                PSSn::<TelescopeError>::builder(),
            )),
        ])
        .build()?
        .into_arcx();

    (*bench.lock().await).update();
    let pssn_e = (*bench.lock().await).pssn.as_mut().unwrap().estimates();
    println!("PSSn: {pssn_e:?}");

    let mut on_axis = Actor::<_, 1, EXPOSURE_RATE>::new(bench.clone()).name("ON-AXIS GMT");

    let n_step = (sim_duration * sim_sampling_frequency as f64) as usize;
    let mut gmt_state: Initiator<_> = Into::<GmtState>::into((
        Arrow::from_parquet("grim.parquet")?,
        CFD_DELAY * sim_sampling_frequency,
        Some(n_step),
    ))
    .into();

    gmt_state
        .add_output()
        .build::<ceo::M1rbm>()
        .into_input(&mut on_axis);
    gmt_state
        .add_output()
        .build::<ceo::M2rbm>()
        .into_input(&mut on_axis);
    gmt_state
        .add_output()
        .build::<ceo::M1modes>()
        .into_input(&mut on_axis);

    let logs = Arrow::builder(1)
        .filename("bench.parquet")
        .build()
        .into_arcx();

    let mut logger = Terminator::<_, EXPOSURE_RATE>::new(logs.clone()).name("Logs");
    on_axis
        .add_output()
        .build::<ceo::WfeRms>()
        .log(&mut logger)
        .await
        .confirm()?
        .add_output()
        .build::<ceo::TipTilt>()
        .log(&mut logger)
        .await
        .confirm()?
        .add_output()
        .build::<ceo::SegmentWfeRms>()
        .log(&mut logger)
        .await
        .confirm()?
        .add_output()
        .build::<ceo::SegmentPiston>()
        .log(&mut logger)
        .await
        .confirm()?
        .add_output()
        .build::<ceo::SegmentTipTilt>()
        .log(&mut logger)
        .await
        .confirm()?
        .add_output()
        .build::<ceo::PSSn>()
        .log(&mut logger)
        .await
        .confirm()?
        .add_output()
        .build::<ceo::DetectorFrame>()
        .logn(&mut logger, 512 * 512)
        .await;

    let model = Model::new(vec![
        Box::new(gmt_state),
        Box::new(on_axis),
        Box::new(logger),
    ])
    .name("bench")
    .check()?
    .flowchart()
    .run();

    let progress = Arc::new(Mutex::new(Progress::new()));
    let onaxis_progress = progress.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_secs(3));
        let bar: Bar = onaxis_progress
            .lock()
            .await
            .bar(EXPOSURE_RATE, "Bench integration");
        loop {
            interval.tick().await;
            let mut progress = onaxis_progress.lock().await;
            let n = (*bench.lock().await).sensor.as_ref().unwrap().n_frame();
            progress.set_and_draw(&bar, n);
        }
    });

    model.wait().await?;

    let pssn: Vec<Vec<f64>> = (*logs.lock().await).get("PSSn")?;
    println!("PPSn: {pssn:?}");
    Ok(())
}
