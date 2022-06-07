use crseo::prelude::*;
use dos_actors::{
    clients::{arrow_client::Arrow, ceo},
    prelude::*,
};
use skyangle::Conversion;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();
    let sim_sampling_frequency = 200_usize;
    let sim_duration = 900.;
    log::info!("Simulation duration: {:6.3}s", sim_duration);

    let atm_duration = 20f32;
    let atm_n_duration = Some((sim_duration / atm_duration as f64).ceil() as i32);
    let atm_sampling = 48 * 16 + 1;
    let atm = Atmosphere::builder().ray_tracing(
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
            .options(vec![ceo::OpticalModelOptions::Atmosphere {
                builder: atm.clone(),
                time_step: tau,
            }])
            .build()?,
        "ON-AXIS GMT",
    )
        .into();

    let n_step = (sim_duration * sim_sampling_frequency as f64) as usize;
    let mut timer: Initiator<_> = Timer::new(n_step).progress().into();

    timer.add_output().build::<Tick>().into_input(&mut on_axis);

    let logs = Arrow::builder(n_step)
        .filename("onaxis.parquet")
        .build()
        .into_arcx();

    let mut logger = Terminator::new(logs.clone()).name("Logs");
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
        .confirm()?;

    Model::new(vec![Box::new(timer), Box::new(on_axis), Box::new(logger)])
        .name("onaxis")
        .check()?
        .flowchart()
        .run()
        .wait()
        .await?;

    Ok(())
}
