use dos_actors::{
    clients::{
        arrow_client::Arrow,
        ceo::M1modes,
        mount::{Mount, MountEncoders, MountSetPoint, MountTorques},
        windloads,
    },
    prelude::*,
};
use fem::{
    dos::{DiscreteModalSolver, ExponentialMatrix},
    fem_io::*,
    FEM,
};
use grim::{agws, M1System, M2System, AGWS};
use nalgebra as na;
use parse_monitors::cfd;
use serde::Deserialize;
use std::{
    collections::HashMap,
    env,
    fs::{create_dir, File},
    path::Path,
};

#[derive(Deserialize, Debug)]
#[allow(dead_code)]
pub struct MonteCarlo {
    index: usize,
    #[serde(rename = "elevation[deg]")]
    elevation: f64,
    #[serde(rename = "azimuth[deg]")]
    azimuth: f64,
    #[serde(rename = "wind[m/s]")]
    wind_velocity: f64,
    #[serde(rename = "r0[m]")]
    r0: f64,
    #[serde(rename = "CFD case")]
    cfd_case: String,
    #[serde(rename = "FEM elevation")]
    fem_elevation: i32,
}

fn fig_2_mode(sid: u32) -> na::DMatrix<f64> {
    let root_env = env::var("M1CALIBRATION").unwrap_or_else(|_| ".".to_string());
    let root = Path::new(&root_env);
    let fig_2_mode: Vec<f64> =
        bincode::deserialize_from(File::open(root.join(format!("m1s{sid}fig2mode.bin"))).unwrap())
            .unwrap();
    if sid < 7 {
        na::DMatrix::from_vec(162, 602, fig_2_mode)
    } else {
        na::DMatrix::from_vec(151, 579, fig_2_mode).insert_rows(151, 11, 0f64)
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let data_repo_env = env::var("DATA_REPO").expect("DATA_REPO env var is not set");
    let data_repo = Path::new(&data_repo_env);
    let job_idx = env::var("AWS_BATCH_JOB_ARRAY_INDEX")
        .expect("AWS_BATCH_JOB_ARRAY_INDEX env var missing")
        .parse::<usize>()
        .expect("AWS_BATCH_JOB_ARRAY_INDEX parsing failed");

    let mut rdr = csv::Reader::from_path(data_repo.join("monte-carlo.csv"))?;
    let monte_carlo: MonteCarlo = rdr.deserialize().nth(job_idx).expect(&format!(
        "failed to load Monte-Carlo parameters #{}",
        job_idx
    ))?;
    println!("Monte-Carlo parameters:");
    println!("{:#?}", monte_carlo);

    /*     env::set_var(
           "FEM_REPO",
           format!(
               "/fsx/MT_mount_zen_{:02}_m1HFN_FSM",
               90 - monte_carlo.fem_elevation
           ),
       );
    */
    let data_path = data_repo.join(format!("monte-carlo_{:04}", job_idx));
    create_dir(&data_path)?;
    println!("Data repository: {:?}", &data_path);
    env::set_var("DATA_REPO", data_path);

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
    log::info!("Simulation duration: {:6.3}s", sim_duration);

    let mut meta_data: HashMap<String, String> = HashMap::new();
    meta_data.insert(
        "sim_sampling_frequency".to_string(),
        format!("{sim_sampling_frequency}"),
    );
    meta_data.insert("CFD_DELAY".to_string(), format!("{CFD_DELAY}"));
    meta_data.insert("M1_RATE".to_string(), format!("{M1_RATE}"));
    meta_data.insert("SH48_RATE".to_string(), format!("{SH48_RATE}"));
    meta_data.insert("FSM_RATE".to_string(), format!("{FSM_RATE}"));
    meta_data.insert("SH48_N_STEP".to_string(), format!("{n_sh48_exposure}"));

    let (cfd_loads, state_space) = {
        use dos_actors::clients::windloads::WindLoads::*;
        let loads = vec![
            TopEnd,
            M2Baffle,
            Trusses,
            M1Baffle,
            MirrorCovers,
            LaserGuideStars,
            CRings,
            GIR,
            Platforms,
        ];
        let mut fem = FEM::from_env()?.static_from_env()?;
        let n_io = (fem.n_inputs(), fem.n_outputs());
        //println!("{}", fem);
        //println!("{}", fem);
        //let cfd_case = cfd::CfdCase::<2021>::colloquial(30, 0, "os", 7)?;

        let cfd_loads =
            windloads::CfdLoads::foh(cfd_path.to_str().unwrap(), sim_sampling_frequency)
                .duration(sim_duration as f64)
                //.time_range((200f64, 340f64))
                //.nodes(loads.iter().flat_map(|x| x.keys()).collect(), locations)
                .loads(loads, &mut fem, 0)
                .m1_segments()
                .m2_segments()
                .build()
                .unwrap()
                .into_arcx();

        (cfd_loads, {
            /*
                    let mut dms = DiscreteModalSolver::<ExponentialMatrix>::from_fem(FEM::from_env()?)
                        .sampling(sim_sampling_frequency as f64)
                        .proportional_damping(2. / 100.)
                        .ins::<CFD2021106F>()
                        .ins::<OSSElDriveTorque>()
                        .ins::<OSSAzDriveTorque>()
                        .ins::<OSSRotDriveTorque>()
                        .ins::<OSSHarpointDeltaF>()
                        .ins::<M1ActuatorsSegment1>()
                        .ins::<M1ActuatorsSegment2>()
                        .ins::<M1ActuatorsSegment3>()
                        .ins::<M1ActuatorsSegment4>()
                        .ins::<M1ActuatorsSegment5>()
                        .ins::<M1ActuatorsSegment6>()
                        .ins::<M1ActuatorsSegment7>()
                        .ins::<MCM2SmHexF>()
                        .ins::<MCM2PZTF>()
                        .outs::<OSSAzEncoderAngle>()
                        .outs::<OSSElEncoderAngle>()
                        .outs::<OSSRotEncoderAngle>()
                        .outs::<OSSHardpointD>()
                        .outs::<OSSM1Lcl>()
                        .outs::<MCM2Lcl6D>()
                        .outs_with::<M1Segment1AxialD>(fig_2_mode(1))
                        .outs_with::<M1Segment2AxialD>(fig_2_mode(2))
                        .outs_with::<M1Segment3AxialD>(fig_2_mode(3))
                        .outs_with::<M1Segment4AxialD>(fig_2_mode(4))
                        .outs_with::<M1Segment5AxialD>(fig_2_mode(5))
                        .outs_with::<M1Segment6AxialD>(fig_2_mode(6))
                        .outs_with::<M1Segment7AxialD>(fig_2_mode(7))
                        .outs::<MCM2SmHexD>()
                        .outs::<MCM2PZTD>();
                    let hsv = dms.hankel_singular_values()?;
                    serde_pickle::to_writer(
                        &mut File::create("hankel_singular_values.pkl")?,
                        &hsv,
                        Default::default(),
                    )?;
            */
            DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
                .sampling(sim_sampling_frequency as f64)
                .proportional_damping(2. / 100.)
                .truncate_hankel_singular_values(1e-4)
                //.max_eigen_frequency(75.)
                //.use_static_gain_compensation(n_io)
                .ins::<CFD2021106F>()
                .ins::<OSSElDriveTorque>()
                .ins::<OSSAzDriveTorque>()
                .ins::<OSSRotDriveTorque>()
                .ins::<OSSHarpointDeltaF>()
                .ins::<M1ActuatorsSegment1>()
                .ins::<M1ActuatorsSegment2>()
                .ins::<M1ActuatorsSegment3>()
                .ins::<M1ActuatorsSegment4>()
                .ins::<M1ActuatorsSegment5>()
                .ins::<M1ActuatorsSegment6>()
                .ins::<M1ActuatorsSegment7>()
                .ins::<MCM2SmHexF>()
                .ins::<MCM2PZTF>()
                .outs::<OSSAzEncoderAngle>()
                .outs::<OSSElEncoderAngle>()
                .outs::<OSSRotEncoderAngle>()
                .outs::<OSSHardpointD>()
                .outs::<OSSM1Lcl>()
                .outs::<MCM2Lcl6D>()
                .outs_with::<M1Segment1AxialD>(fig_2_mode(1))
                .outs_with::<M1Segment2AxialD>(fig_2_mode(2))
                .outs_with::<M1Segment3AxialD>(fig_2_mode(3))
                .outs_with::<M1Segment4AxialD>(fig_2_mode(4))
                .outs_with::<M1Segment5AxialD>(fig_2_mode(5))
                .outs_with::<M1Segment6AxialD>(fig_2_mode(6))
                .outs_with::<M1Segment7AxialD>(fig_2_mode(7))
                .outs::<MCM2SmHexD>()
                .outs::<MCM2PZTD>()
                .build()?
                .into_arcx()
        })
    };
    println!("{}", *state_space.lock().await);
    //println!("Y sizes: {:?}", state_space.y_sizes);

    let n_step = (sim_duration * sim_sampling_frequency as f64) as usize;
    let logging = Arrow::builder(n_step)
        .metadata(meta_data)
        .filename("grim.parquet")
        .build()
        .into_arcx();
    //let mnt_ctrl = Mount::at_zenith_angle(90 - monte_carlo.fem_elevation)?.into_arcx();
    let mnt_ctrl = Mount::at_zenith_angle(30)?.into_arcx();

    (*cfd_loads.lock().await).stop_after(CFD_DELAY * sim_sampling_frequency);

    let model_1 = {
        let mut source: Initiator<_> = Actor::new(cfd_loads.clone());
        let mut sink = Terminator::<_>::new(logging.clone());
        // FEM
        let mut fem: Actor<_> = Actor::new(state_space.clone());
        // MOUNT
        let mut mount: Actor<_> = Actor::new(mnt_ctrl.clone());

        /*         let mut cfd_202110_6f: Initiator<_> = Signals::new(
            <CfdLoads<windloads::FOH> as dos_actors::Size<CFD2021106F>>::len(
                &*cfd_loads.lock().await,
            ),
            CFD_DELAY * sim_sampling_frequency,
        )
        .into();
        let mut oss_m1_lcl_6f: Initiator<_> = Signals::new(
            <CfdLoads<windloads::FOH> as dos_actors::Size<OSSM1Lcl6F>>::len(
                &*cfd_loads.lock().await,
            ),
            CFD_DELAY * sim_sampling_frequency,
        )
        .into();
        let mut mc_m2_lcl_6f: Initiator<_> = Signals::new(
            <CfdLoads<windloads::FOH> as dos_actors::Size<MCM2LclForce6F>>::len(
                &*cfd_loads.lock().await,
            ),
            CFD_DELAY * sim_sampling_frequency,
        )
        .into(); */

        source
            .add_output()
            .build::<CFD2021106F>()
            .into_input(&mut fem);
        source
            .add_output()
            .build::<OSSM1Lcl6F>()
            .into_input(&mut fem);
        source
            .add_output()
            .build::<MCM2LclForce6F>()
            .into_input(&mut fem);

        let mut mount_set_point: Initiator<_> = Signals::new(3, n_step).into();
        mount_set_point
            .add_output()
            .build::<MountSetPoint>()
            .into_input(&mut mount);
        mount
            .add_output()
            .build::<MountTorques>()
            .into_input(&mut fem);

        fem.add_output()
            .bootstrap()
            .build::<MountEncoders>()
            .into_input(&mut mount);
        fem.add_output()
            .bootstrap()
            .build::<OSSM1Lcl>()
            .logn(&mut sink, 42)
            .await;
        fem.add_output()
            .bootstrap()
            .build::<MCM2Lcl6D>()
            .logn(&mut sink, 42)
            .await;
        fem.add_output()
            .bootstrap()
            .build::<M1modes>()
            .logn(&mut sink, 162 * 7)
            .await;

        Model::new(vec![
            Box::new(source),
            Box::new(mount_set_point),
            Box::new(fem),
            Box::new(mount),
            Box::new(sink),
        ])
        .flowchart()
        .check()?
        .run()
    };

    #[cfg(not(feature = "full"))]
    model_1.wait().await?;

    #[cfg(feature = "full")]
    {
        use crseo::{calibrations, Atmosphere, FromBuilder, Gmt};
        use dos_actors::{
            clients::{
                ceo,
                ceo::M1modes,
                fsm::*,
                m1::*,
                mount::{MountEncoders, MountSetPoint, MountTorques},
                Integrator,
            },
            prelude::*,
        };
        use linya::{Bar, Progress};
        use lom::{Loader, LoaderTrait, OpticalSensitivities, OpticalSensitivity};
        use skyangle::Conversion;
        use std::{fs::File, path::Path, sync::Arc, time::Duration};
        use tokio::sync::Mutex;

        let mut source: Initiator<_> = Actor::new(cfd_loads.clone()).name("CFD Loads");
        let mut sink = Terminator::<_>::new(logging.clone()).name("GMT State");
        // FEM
        let mut fem: Actor<_> = Actor::new(state_space.clone()).name("GMT Finite Element Model");
        // MOUNT
        let mut mount: Actor<_> = Actor::new(mnt_ctrl.clone()).name("Mount Control");

        /*         let mut cfd_202110_6f: Initiator<_> = Signals::new(
            <CfdLoads<windloads::FOH> as dos_actors::Size<CFD2021106F>>::len(
                &*cfd_loads.lock().await,
            ),
            n_sh48_exposure * SH48_RATE,
        )
        .into();
        let mut oss_m1_lcl_6f: Initiator<_> = Signals::new(
            <CfdLoads<windloads::FOH> as dos_actors::Size<OSSM1Lcl6F>>::len(
                &*cfd_loads.lock().await,
            ),
            n_sh48_exposure * SH48_RATE,
        )
        .into();
        let mut mc_m2_lcl_6f: Initiator<_> = Signals::new(
            <CfdLoads<windloads::FOH> as dos_actors::Size<MCM2LclForce6F>>::len(
                &*cfd_loads.lock().await,
            ),
            n_sh48_exposure * SH48_RATE,
        )
        .into(); */
        source
            .add_output()
            .build::<CFD2021106F>()
            .into_input(&mut fem);
        source
            .add_output()
            .build::<OSSM1Lcl6F>()
            .into_input(&mut fem);
        source
            .add_output()
            .build::<MCM2LclForce6F>()
            .into_input(&mut fem);

        let mut mount_set_point: Initiator<_> = (Signals::new(3, n_step), "Mount 0pt").into();
        mount_set_point
            .add_output()
            .build::<MountSetPoint>()
            .into_input(&mut mount);
        mount
            .add_output()
            .build::<MountTorques>()
            .into_input(&mut fem);

        fem.add_output()
            .bootstrap()
            .build::<MountEncoders>()
            .into_input(&mut mount);

        // M1 SYSTEM
        let mut m1_actors = M1System::<M1_RATE, SH48_RATE>::new(n_step, 27)?;
        // M2 SYSTEM
        let mut m2_actors = M2System::<FSM_RATE>::new(n_step);
        // AGWS OPTICAL MODEL
        println!("AGWS");

        let mut agws = AGWS::new().gmt(Gmt::builder().m1_n_mode(162));
        if let Ok(_) = env::var("WITH_ATMOSPHERE") {
            println!("WITH_ATMOSPHERE");
            let atm_duration = 20f32;
            let atm_n_duration = Some((sim_duration / atm_duration as f64).ceil() as i32);
            let atm_sampling = 48 * 16 + 1;
            let atm = Atmosphere::builder()
                .ray_tracing(
                    25.5,
                    atm_sampling,
                    20f32.from_arcmin(),
                    atm_duration,
                    Some("/fsx/atmosphere/free_atm_15mn.bin".to_owned()),
                    atm_n_duration,
                )
                .remove_turbulence_layer(0)
                .r0_at_zenith(monte_carlo.r0);
            let tau = (sim_sampling_frequency as f64).recip();
            agws = agws.atmosphere(atm, tau);
        }
        if let Ok(_) = env::var("WITH_DOME_SEEING") {
            println!("WITH_DOME_SEEING");
            agws = agws.dome_seeing(
                cfd_path.to_str().unwrap().to_string(),
                (sim_sampling_frequency / 5) as usize,
            );
        }
        if let Ok(var) = env::var("WITH_STATIC_ABERRATION") {
            println!("WITH_STATIC_ABERRATION: {var}");
            let static_phase: Vec<f32> = bincode::deserialize_from(File::open(var)?)?;
            agws = agws.static_aberration(static_phase);
        }
        let senses: OpticalSensitivities = Loader::<OpticalSensitivities>::default().load()?;
        let rxy_2_stt = senses[OpticalSensitivity::SegmentTipTilt(Vec::new())].m2_rxy()?;
        use calibrations::Mirror;
        use calibrations::Segment::*;
        let sh24 = agws::AgwsShackHartmann::<agws::SH24, agws::Geometric, FSM_RATE>::builder(1)
            .guide_star(
                crseo::Source::builder()
                    .zenith_azimuth(vec![6f32.from_arcmin()], vec![30f32.to_radians()]),
            )
            .flux_threshold(0.8)
            .poker(vec![
                Some(vec![(Mirror::M2, vec![Rxyz(1e-6, Some(0..2))])]);
                7
            ])
            .left_pinv(rxy_2_stt);
        let n_sh48 = 1;
        let sh48 =
            agws::AgwsShackHartmann::<agws::SH48, agws::Geometric, SH48_RATE>::builder(n_sh48)
                .guide_star(
                    crseo::Source::builder()
                        .zenith_azimuth(vec![8f32.from_arcmin()], vec![120f32.to_radians()]),
                )
                .flux_threshold(0.8)
                .poker(vec![
                    Some(vec![(Mirror::M1MODES, vec![Modes(1e-6, 0..27)])]);
                    7
                ]);
        let (mut agws_tt7, mut agws_sh48) = agws.build(Some(sh24), Some(sh48)).into();
        agws_tt7
            .add_output()
            .build::<TTFB>()
            .into_input(&mut m2_actors.tiptilt());

        fem.add_output()
            .bootstrap()
            .multiplex(3)
            .unbounded()
            .build::<OSSM1Lcl>()
            .into_input(&mut agws_tt7)
            .into_input(&mut agws_sh48)
            .into_input(&mut sink)
            .confirm()?
            .add_output()
            .bootstrap()
            .multiplex(3)
            .unbounded()
            .build::<MCM2Lcl6D>()
            .into_input(&mut agws_tt7)
            .into_input(&mut agws_sh48)
            .into_input(&mut sink)
            .confirm()?
            .add_output()
            .bootstrap()
            .multiplex(3)
            .unbounded()
            .build::<M1modes>()
            .into_input(&mut agws_tt7)
            .into_input(&mut agws_sh48)
            .into_input(&mut sink)
            .confirm()?;

        let zero_point = vec![0f64; 27 * 7];
        let mut integrator: Actor<_, SH48_RATE, SH48_RATE> =
            Integrator::<f64, ceo::SensorData>::new(27 * 7)
                .gain(0.5)
                .zero(zero_point)
                .into();

        agws_sh48
            .add_output()
            .build::<ceo::SensorData>()
            .into_input(&mut integrator);

        m1_actors
            .segments()
            .fold(
                integrator
                    .add_output()
                    .bootstrap()
                    .multiplex(7)
                    .build::<M1ModalCmd>(),
                |x, s| s.add_input(x),
            )
            .confirm()?;

        model_1.wait().await?;

        (*cfd_loads.lock().await).start_from(CFD_DELAY * sim_sampling_frequency);

        let mut m1_actors = m1_actors.build(&mut fem);
        let mut m2_actors = m2_actors.build(&mut fem);
        let mut actors: Vec<Box<dyn Task>> = vec![
            Box::new(source),
            Box::new(mount_set_point),
            Box::new(mount),
            Box::new(agws_tt7),
            Box::new(agws_sh48),
            Box::new(integrator),
            Box::new(fem),
            Box::new(sink),
        ];
        actors.append(&mut m1_actors);
        actors.append(&mut m2_actors);
        let model = Model::new(actors)
            .name("monte-carlo")
            .flowchart()
            .check()?
            .run();

        let logs = logging.clone();
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
    }

    Ok(())
}
