use std::{env, fs::File, path::Path};

use crseo::{Atmosphere, FromBuilder, Gmt};
use dos_actors::{
    clients::{
        arrow_client::Arrow,
        ceo::{
            M1modes, M1rbm, M2rbm, OpticalModel, OpticalModelOptions, PSSnFwhm, PSSnOptions,
            SegmentTipTilt, SegmentWfe, Wavefront, WfeRms,
        },
        gmt_state::GmtState,
    },
    prelude::*,
};
use monte_carlo::MonteCarlo;
use parse_monitors::cfd;
use skyangle::Conversion;
use vec_box::vec_box;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let data_repo_env = env::var("DATA_REPO").expect("DATA_REPO env var is not set");
    let data_repo = Path::new(&data_repo_env);
    let job_idx = env::var("AWS_BATCH_JOB_ARRAY_INDEX")
        .expect("AWS_BATCH_JOB_ARRAY_INDEX env var missing")
        .parse::<usize>()
        .expect("AWS_BATCH_JOB_ARRAY_INDEX parsing failed");

    let monte_carlo = MonteCarlo::new(data_repo.join("monte-carlo.csv"), job_idx % 100)?;
    println!("Monte-Carlo parameters:");
    println!("{monte_carlo}");

    let data_path = data_repo.join(format!("monte-carlo_{:04}", job_idx));
    println!("Data repository: {:?}", &data_path);

    let cfd_case = monte_carlo.cfd_case;
    let cfd_path = cfd::Baseline::<2021>::path().join(cfd_case.clone());

    let sim_sampling_frequency = 1000_usize;

    const CFD_RATE: usize = 1;
    const CFD_DELAY: usize = 10; // seconds
    let cfd_sampling_frequency = sim_sampling_frequency / CFD_RATE;
    println!("CFD CASE ({}Hz): {}", cfd_sampling_frequency, cfd_case);

    const M1_RATE: usize = 10;
    assert_eq!(sim_sampling_frequency / M1_RATE, 100); // Hz

    const SH48_RATE: usize = 30_000;
    assert_eq!(SH48_RATE / sim_sampling_frequency, 30); // Seconds

    const FSM_RATE: usize = 5;
    assert_eq!(sim_sampling_frequency / FSM_RATE, 200); // Hz

    let n_sh48_exposure = env::var("SH48_N_STEP")?.parse::<usize>()?;
    let sim_duration = (CFD_DELAY + n_sh48_exposure * SH48_RATE / sim_sampling_frequency) as f64;
    log::info!(
        "Simulation duration: {:6.3}s",
        n_sh48_exposure * SH48_RATE / sim_sampling_frequency
    );
    let n_step = n_sh48_exposure * SH48_RATE;

    let mut gmt_state: Initiator<_> = Into::<GmtState>::into((
        Arrow::from_parquet(data_path.join("grim.parquet"))?,
        CFD_DELAY * sim_sampling_frequency,
        Some(n_step),
    ))
    .into();

    env::set_var("DATA_REPO", &data_path);

    let mut options_ref = vec![];
    let mut options = vec![];
    if let Ok(_) = env::var("WITH_ATMOSPHERE") {
        println!("WITH_ATMOSPHERE");
        let atm_duration = 20f32;
        let atm_n_duration = Some((sim_duration / atm_duration as f64).ceil() as i32);
        let atm_sampling = 48 * 16 + 1;
        let free_atm = Atmosphere::builder()
            .ray_tracing(
                25.5,
                atm_sampling,
                20f32.from_arcmin(),
                atm_duration,
                Some("/fsx/atmosphere/free_atm_15mn.bin".to_owned()),
                atm_n_duration,
            )
            .remove_turbulence_layer(0)
            .r0_at_zenith(monte_carlo.r0)
            .zenith_angle((90f64 - monte_carlo.elevation).to_radians());
        let atm = Atmosphere::builder()
            .ray_tracing(
                25.5,
                atm_sampling,
                20f32.from_arcmin(),
                atm_duration,
                Some("/fsx/atmosphere/atm_15mn.bin".to_owned()),
                atm_n_duration,
            )
            .r0_at_zenith(monte_carlo.r0)
            .zenith_angle((90f64 - monte_carlo.elevation).to_radians());
        let tau = (sim_sampling_frequency as f64).recip();
        options.push(OpticalModelOptions::Atmosphere {
            builder: free_atm,
            time_step: tau,
        });

        options_ref.push(OpticalModelOptions::Atmosphere {
            builder: atm,
            time_step: tau,
        })
    }
    if let Ok(var) = env::var("WITH_STATIC_ABERRATION") {
        println!("WITH_STATIC_ABERRATION: {var}");
        let static_phase: Vec<f32> = bincode::deserialize_from(File::open(var)?)?;
        options.push(OpticalModelOptions::StaticAberration(static_phase.into()))
    };
    if let Ok(_) = env::var("WITH_DOME_SEEING") {
        println!("WITH_DOME_SEEING");
        options.push(OpticalModelOptions::DomeSeeing {
            cfd_case: cfd_path.to_str().unwrap().to_string(),
            upsampling_rate: (sim_sampling_frequency / 5) as usize,
        })
    }
    let pssn = OpticalModelOptions::PSSn(PSSnOptions::AtmosphereTelescope(crseo::PSSn::builder()));
    options.push(pssn.clone());
    options_ref.push(pssn);

    let photometry = env::var("PHOTOMETRY").unwrap_or("V".to_string());
    println!("Photometry: {photometry} band");
    let src = crseo::Source::builder()
        .pupil_sampling(48 * 16 + 1)
        .band(photometry.as_str())
        .field_delaunay21();
    println!("Sources (zenith[arcmin],azimuth[degree]):");
    src.zenith
        .iter()
        .zip(src.azimuth.iter())
        .enumerate()
        .for_each(|(k, (&z, &a))| {
            println!("{:02}.{:6.2},{:8.2}", k, z.to_arcmin(), a.to_degrees())
        });

    let mut actors: Vec<Box<dyn Task>> = Vec::new();

    let mut timer: Initiator<_> = Timer::new(n_step).progress().into();
    let mut timer_output = timer
        .add_output()
        .multiplex(
            env::var("WITH_REFERENCE_MODEL").map_or(0, |_| 1)
                + env::var("WITH_SIMULATED_MODEL").map_or(0, |_| 1),
        )
        .build::<Tick>();

    // Reference model
    let on_axis_ref_mdl = if let Ok(_) = env::var("WITH_REFERENCE_MODEL") {
        println!("REFERENCE MODEL");
        let on_axis_ref_mdl = OpticalModel::builder()
            .source(src.clone())
            .options(options_ref)
            .build()?
            .into_arcx();
        let mut on_axis_ref: Actor<_> = Actor::new(on_axis_ref_mdl.clone()).name("Ref. On-Axis");
        let mut logs_ref: Terminator<_> = (
            Arrow::builder(n_step)
                .filename(format!("ref-{photometry}"))
                .build(),
            "Ref. Logs",
        )
            .into();

        timer_output = timer_output.into_input(&mut on_axis_ref);
        on_axis_ref
            .add_output()
            .unbounded()
            .build::<WfeRms>()
            .log(&mut logs_ref)
            .await;
        on_axis_ref
            .add_output()
            .unbounded()
            .build::<SegmentWfe>()
            .log(&mut logs_ref)
            .await;
        on_axis_ref
            .add_output()
            .unbounded()
            .build::<SegmentTipTilt>()
            .log(&mut logs_ref)
            .await;

        actors.append(&mut vec_box![logs_ref, on_axis_ref]);
        Some(on_axis_ref_mdl)
    } else {
        None
    };

    // Actual simulated model
    let on_axis_mdl = if let Ok(_) = env::var("WITH_SIMULATED_MODEL") {
        println!("SIMULATED MODEL");
        let on_axis_mdl = OpticalModel::builder()
            .gmt(Gmt::builder().m1_n_mode(162))
            .source(src)
            .options(options)
            .build()?
            .into_arcx();
        let mut on_axis: Actor<_> = Actor::new(on_axis_mdl.clone()).name("Sim. On-Axis");

        let mut logs: Terminator<_> = (
            Arrow::builder(n_step)
                .filename(format!("sim-{photometry}"))
                .build(),
            "Sim. Logs",
        )
            .into();

        gmt_state
            .add_output()
            .build::<M1rbm>()
            .into_input(&mut on_axis)
            .confirm()?
            .add_output()
            .build::<M2rbm>()
            .into_input(&mut on_axis)
            .confirm()?
            .add_output()
            .build::<M1modes>()
            .into_input(&mut on_axis)
            .confirm()?;
        timer_output = timer_output.into_input(&mut on_axis);
        on_axis.add_output().build::<WfeRms>().log(&mut logs).await;
        on_axis
            .add_output()
            .build::<SegmentWfe>()
            .log(&mut logs)
            .await;
        on_axis
            .add_output()
            .build::<SegmentTipTilt>()
            .log(&mut logs)
            .await;

        actors.append(&mut vec_box![logs, gmt_state, on_axis,]);
        Some(on_axis_mdl)
    } else {
        None
    };
    timer_output.confirm()?;
    actors.push(Box::new(timer));

    if actors.is_empty() {
        panic!("No actors found, select actors by setting the environment variable WITH_REFERENCE_MODE and WITH_SIMULATED_MODEL")
    }

    Model::new(actors)
        .name("monte-carlo-bench")
        .check()?
        .flowchart()
        .run()
        .wait()
        .await?;

    let mut timer: Initiator<_> = Timer::new(0).into();
    let mut timer_output = timer
        .add_output()
        .multiplex(
            env::var("WITH_REFERENCE_MODEL").map_or(0, |_| 1)
                + env::var("WITH_SIMULATED_MODEL").map_or(0, |_| 1),
        )
        .build::<Tick>();

    let mut actors: Vec<Box<dyn Task>> = Vec::new();

    if let Ok(_) = env::var("WITH_REFERENCE_MODEL") {
        let mut on_axis_ref: Actor<_> =
            Actor::new(on_axis_ref_mdl.as_ref().unwrap().clone()).name("Ref. On-Axis");
        let mut logs_ref: Terminator<_> = (
            Arrow::builder(1).filename("ref_pssn-fwhm").build(),
            "Ref. Logs",
        )
            .into();
        timer_output = timer_output.into_input(&mut on_axis_ref);
        on_axis_ref
            .add_output()
            .bootstrap()
            .build::<PSSnFwhm>()
            .log(&mut logs_ref)
            .await;
        on_axis_ref
            .add_output()
            .bootstrap()
            .build::<Wavefront>()
            .log(&mut logs_ref)
            .await;
        actors.append(&mut vec_box![logs_ref, on_axis_ref,]);
    }
    if let Ok(_) = env::var("WITH_SIMULATED_MODEL") {
        let mut on_axis: Actor<_> =
            Actor::new(on_axis_mdl.as_ref().unwrap().clone()).name("Sim. On-Axis");
        let mut logs: Terminator<_> = (
            Arrow::builder(1).filename("logged_pssn-fwhm").build(),
            "Sim. Logs",
        )
            .into();
        timer_output = timer_output.into_input(&mut on_axis);
        on_axis
            .add_output()
            .bootstrap()
            .build::<PSSnFwhm>()
            .log(&mut logs)
            .await;
        on_axis
            .add_output()
            .bootstrap()
            .build::<Wavefront>()
            .log(&mut logs)
            .await;
        actors.append(&mut vec_box![logs, on_axis,]);
    }
    timer_output.confirm()?;
    actors.push(Box::new(timer));

    Model::new(actors)
        .name("monte-carlo-bench-final")
        .check()?
        .flowchart()
        .run()
        .wait()
        .await?;

    Ok(())
}
