use crseo::{
    calibrations::{Mirror, Segment::*},
    FromBuilder, Source,
};
use grim::agws;
use skyangle::Conversion;
use std::fs::File;

fn main() -> anyhow::Result<()> {
    let rxy = Rxyz(1e-6, Some(0..2));
    let rxyz = Rxyz(1e-6, Some(0..3));
    let txy = Txyz(1e-6, Some(0..2));
    let txyz = Txyz(1e-6, Some(0..3));

    for senses in vec![
        (Mirror::M1, vec![txyz, rxy.clone()]),
        (Mirror::M2, vec![rxy]),
    ] {
        {
            // SH 24
            println!("SH24");
            let rho = 6f32.from_arcmin();
            let pokes = (1..=1)
                .map(|i| {
                    let gs = Source::builder()
                        .size(i)
                        .zenith_azimuth(vec![rho], vec![60f32.to_radians()]);
                    let sh24 =
                        agws::AgwsShackHartmann::<agws::SH24, agws::Geometric, 1>::builder(i)
                            .guide_star(gs)
                            .flux_threshold(0.8)
                            .poker(vec![Some(vec![senses.clone()]); 7]);
                    let sys = agws::AGWS::new();
                    sys.poke_with(&sh24).map(|mat| mat.as_slice().to_vec())
                })
                .collect::<grim::Result<Vec<Vec<f64>>>>()?;

            serde_pickle::to_writer(
                &mut File::create(format!("poke24_{:?}.pkl", senses.0.clone()))?,
                &pokes[0],
                Default::default(),
            )?;
        }

        {
            // SH 48
            println!("SH48");
            let rho = 8f32.from_arcmin();
            let pokes = (1..=3)
                .map(|i| {
                    let gs = Source::builder().size(i).on_ring(rho);
                    let sh48 =
                        agws::AgwsShackHartmann::<agws::SH48, agws::Geometric, 1>::builder(i)
                            .guide_star(gs)
                            .flux_threshold(0.8)
                            .poker(vec![Some(vec![senses.clone(),]); 7]);
                    let sys = agws::AGWS::new();
                    sys.poke_with(&sh48).map(|mat| mat.as_slice().to_vec())
                })
                .collect::<grim::Result<Vec<Vec<f64>>>>()?;

            serde_pickle::to_writer(
                &mut File::create(format!("poke48_{:?}.pkl", senses.0.clone()))?,
                &pokes,
                Default::default(),
            )?;
        }
    }

    Ok(())
}
