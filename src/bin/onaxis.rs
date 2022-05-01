use crseo::{Builder, ATMOSPHERE};
use dos_actors::{
    clients::{arrow_client::Arrow, ceo},
    prelude::*,
};
use linya::{Bar, Progress};
use skyangle::Conversion;
use std::{sync::Arc, time::Duration};
use tokio::sync::Mutex;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();
    let sim_sampling_frequency = 200_usize;
    let sim_duration = 900.;
    log::info!("Simulation duration: {:6.3}s", sim_duration);

    let atm_duration = 20f32;
    let atm_n_duration = Some((sim_duration / atm_duration as f64).ceil() as i32);
    let atm_sampling = 48 * 16 + 1;
    let atm = ATMOSPHERE::new().ray_tracing(
        25.5,
        atm_sampling,
        20f32.from_arcmin(),
        atm_duration,
        Some("/fsx/atmosphere/atm_15mn.bin".to_owned()),
        atm_n_duration,
    );
    let tau = (sim_sampling_frequency as f64).recip();
    let mut on_axis: Actor<_> = (
        ceo::OpticalModel::builder()
            .atmosphere(atm)
            .sampling_period(tau)
            .build()?,
        "ON-AXIS GMT",
    )
        .into();

    let n_step = (sim_duration * sim_sampling_frequency as f64) as usize;
    let mut timer: Initiator<_> = Timer::new(n_step).into();

    timer
        .add_output()
        .build::<Void, Tick>()
        .into_input(&mut on_axis);

    let logs = Arrow::builder(n_step)
        .entry::<f64, ceo::WfeRms>(1)
        .entry::<f64, ceo::TipTilt>(2)
        .entry::<f64, ceo::SegmentWfeRms>(7)
        .entry::<f64, ceo::SegmentPiston>(7)
        .entry::<f64, ceo::SegmentTipTilt>(14)
        .filename("onaxis.parquet")
        .build()
        .into_arcx();
    type D = Vec<f64>;
    let mut logger = Terminator::new(logs.clone()).name("Logs");
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

    let model = Model::new(vec![Box::new(timer), Box::new(on_axis), Box::new(logger)])
        .name("onaxis")
        .check()?
        .flowchart()
        .run();

    let progress = Arc::new(Mutex::new(Progress::new()));
    let logging_progress = progress.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_secs(5));
        let bar: Bar = logging_progress.lock().await.bar(n_step, "Logging");
        loop {
            interval.tick().await;
            let mut progress = logging_progress.lock().await;
            progress.set_and_draw(&bar, (*logs.lock().await).size());
            if progress.is_done(&bar) {
                break;
            }
        }
    });

    model.wait().await?;

    Ok(())
}
