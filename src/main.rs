use dos_actors::clients::{
    arrow_client::{Arrow, Get},
    ceo::{DetectorFrame, OpticalModel, OpticalModelOptions, ShackHartmannOptions, WfeRms},
};
use dos_actors::prelude::*;

use crseo::{Builder, GmtBuilder, ShackHartmannBuilder, SourceBuilder};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    /*let mut gmt = GmtBuilder::default().build().unwrap();
    let mut src = SourceBuilder::default().build().unwrap();
    src.through(&mut gmt).xpupil();
    println!("WFE RMS: {}", src.wfe_rms_10e(-9)[0]);*/

    let n_step = 5;
    let mut timer: Initiator<_> = Timer::new(n_step).into();
    let mut optical_model: Actor<_> = (
        OpticalModel::builder()
            .options(vec![OpticalModelOptions::ShackHartmann {
                options: ShackHartmannOptions::Diffractive(
                    ShackHartmannBuilder::builder().detector(128, None, Some(4), None),
                ),
                flux_threshold: 0f64,
            }])
            .build()?,
        "On-axis source",
    )
        .into();
    //let logging = Logging::default().into_arcx();
    let logging = Arrow::builder(n_step).no_save().build().into_arcx();
    let mut logs = Terminator::<_>::new(logging.clone());

    timer
        .add_output()
        .build::<Tick>()
        .into_input(&mut optical_model);
    optical_model
        .add_output()
        .build::<WfeRms>()
        .log(&mut logs)
        .await;
    optical_model
        .add_output()
        .build::<DetectorFrame>()
        .logn(&mut logs, 128 * 128)
        .await;

    Model::new(vec![
        Box::new(timer),
        Box::new(optical_model),
        Box::new(logs),
    ])
    .name("tryout")
    .check()?
    .flowchart()
    .run()
    .wait()
    .await?;

    let data: Vec<Vec<f64>> = (*logging.lock().await).get("WfeRms")?;
    println!("{data:?}");

    let data: Vec<Vec<f32>> =
        (*logging.lock().await).get_skip_take("DetectorFrame", n_step - 1, Some(1))?;
    #[cfg(feature = "complot")]
    {
        let n = 128usize;
        let _: complot::Heatmap = ((data[0].as_slice(), (n, n)), None).into();
    }
    Ok(())
}
