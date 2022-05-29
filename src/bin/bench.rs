use crseo::{pssn, Builder, ShackHartmannBuilder, ATMOSPHERE, GMT, PSSN, SOURCE};
use dos_actors::{
    clients::{arrow_client::Arrow, ceo, gmt_state::GmtState},
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

    let gmt_builder = GMT::new().m1_n_mode(162);
    let bench = ceo::OpticalModel::builder()
        .gmt(gmt_builder)
        .source(SOURCE::new().zenith_azimuth(vec![6f32.from_arcmin()], vec![0f32]))
        .options(vec![
            ceo::OpticalModelOptions::ShackHartmann {
                options: ceo::ShackHartmannOptions::Diffractive(ShackHartmannBuilder::<
                    crseo::Diffractive,
                >::new()),
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
                PSSN::<pssn::TelescopeError>::new(),
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
        n_step,
    ))
    .into();

    type D = Vec<f64>;
    gmt_state
        .add_output()
        .build::<D, ceo::M1rbm>()
        .into_input(&mut on_axis);
    gmt_state
        .add_output()
        .build::<D, ceo::M2rbm>()
        .into_input(&mut on_axis);
    gmt_state
        .add_output()
        .build::<D, ceo::M1modes>()
        .into_input(&mut on_axis);

    let logs = Arrow::builder(1)
        .entry::<f64, ceo::WfeRms>(1)
        .entry::<f64, ceo::TipTilt>(2)
        .entry::<f64, ceo::SegmentWfeRms>(7)
        .entry::<f64, ceo::SegmentPiston>(7)
        .entry::<f64, ceo::SegmentTipTilt>(14)
        .entry::<f64, ceo::PSSn>(1)
        .entry::<f32, ceo::DetectorFrame>(512 * 512)
        .filename("bench.parquet")
        .build()
        .into_arcx();

    let mut logger = Terminator::<_, EXPOSURE_RATE>::new(logs.clone()).name("Logs");
    on_axis
        .add_output()
        .build::<D, ceo::WfeRms>()
        .into_input(&mut logger);
    on_axis
        .add_output()
        .build::<D, ceo::TipTilt>()
        .into_input(&mut logger);
    on_axis
        .add_output()
        .build::<D, ceo::SegmentWfeRms>()
        .into_input(&mut logger);
    on_axis
        .add_output()
        .build::<D, ceo::SegmentPiston>()
        .into_input(&mut logger);
    on_axis
        .add_output()
        .build::<D, ceo::SegmentTipTilt>()
        .into_input(&mut logger);
    on_axis
        .add_output()
        .build::<D, ceo::PSSn>()
        .into_input(&mut logger);
    on_axis
        .add_output()
        .build::<Vec<f32>, ceo::DetectorFrame>()
        .into_input(&mut logger);

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

    let pssn = (*logs.lock().await).get("PSSn")?;
    println!("PPSn: {pssn:?}");
    Ok(())
}
