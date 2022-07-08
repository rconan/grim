use crseo::{
    calibrations, ceo, cu::Single, Builder, Calibration, CrseoError, Cu, Diffractive, FromBuilder,
    Geometric, Gmt, GmtBuilder, WavefrontSensor, WavefrontSensorBuilder, SH48,
};
use serde_pickle as pickle;
use skyangle::Conversion;
use std::fs::File;
use std::time::Instant;

fn main() -> std::result::Result<(), CrseoError> {
    let n_sensor = 1;
    let mut gmt = Gmt::builder().build().unwrap();
    let wfs_blueprint = crseo::SH48::<crseo::Geometric>::new().n_sensor(n_sensor);
    let mut gs = wfs_blueprint
        .guide_stars(None)
        .on_ring(6f32.from_arcmin())
        .build()?;

    let mut wfs = wfs_blueprint.build().unwrap();

    gs.through(&mut gmt).xpupil();
    println!("GS WFE RMS: {}nm", gs.wfe_rms_10e(-9)[0]);
    wfs.calibrate(&mut gs, 0.8);

    let mut gmt2wfs = Calibration::new(
        &gmt,
        &gs,
        crseo::SH48::<crseo::Geometric>::new().n_sensor(n_sensor),
    );
    let mirror = vec![calibrations::Mirror::M2];
    let segments = vec![vec![calibrations::Segment::Rxyz(1e-6, Some(0..2))]; 7];
    let spec = vec![(
        calibrations::Mirror::M2,
        vec![calibrations::Segment::Rxyz(1e-6, Some(0..2))],
    )];
    let now = Instant::now();
    gmt2wfs.calibrate(
        //mirror,
        //segments,
        vec![Some(spec); 7],
        //calibrations::ValidLensletCriteria::OtherSensor(&mut wfs),
        calibrations::ValidLensletCriteria::Threshold(Some(0.8)),
    );
    println!(
        "GMT 2 WFS calibration [{}x{}] in {}s",
        gmt2wfs.n_data,
        gmt2wfs.n_mode,
        now.elapsed().as_secs()
    );
    let poke_sum = gmt2wfs.poke.from_dev().iter().sum::<f64>();
    println!("Poke sum: {}", poke_sum);
    let mut file = File::create("poke.pkl").unwrap();
    pickle::to_writer(&mut file, &gmt2wfs.poke.from_dev(), Default::default()).unwrap();

    Ok(())
}
