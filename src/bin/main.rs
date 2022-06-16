use chrono::prelude::*;
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
use grim::{agws, M1System, AGWS};
use nalgebra as na;
use parse_monitors::cfd;
use std::{
    env,
    fs::{create_dir, File},
    path::Path,
};

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

    let local: DateTime<Local> = Local::now();
    let data_path = Path::new("/fsx").join("grim").join(local.to_rfc3339());
    create_dir(&data_path)?;
    println!("Data repository: {:?}", &data_path);
    env::set_var("DATA_REPO", data_path);

    let sim_sampling_frequency = 1000_usize;

    const CFD_RATE: usize = 1;
    const CFD_DELAY: usize = 10; // seconds
    let cfd_sampling_frequency = sim_sampling_frequency / CFD_RATE;

    const M1_RATE: usize = 10;
    assert_eq!(sim_sampling_frequency / M1_RATE, 100); // Hz

    const SH48_RATE: usize = 30_000;
    assert_eq!(SH48_RATE / sim_sampling_frequency, 30); // Seconds

    const FSM_RATE: usize = 5;
    assert_eq!(sim_sampling_frequency / FSM_RATE, 200); // Hz

    let n_sh48_exposure = env::var("SH48_N_STEP")?.parse::<usize>()?;
    let sim_duration = (CFD_DELAY + n_sh48_exposure * SH48_RATE / sim_sampling_frequency) as f64;
    log::info!("Simulation duration: {:6.3}s", sim_duration);

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
        let cfd_case = cfd::CfdCase::<2021>::colloquial(30, 0, "os", 7)?;
        println!("CFD CASE ({}Hz): {}", cfd_sampling_frequency, cfd_case);
        let cfd_path = cfd::Baseline::<2021>::path().join(cfd_case.to_string());

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
                //.truncate_hankel_singular_values(1e-4)
                //.max_eigen_frequency(75.)
                .use_static_gain_compensation(n_io)
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
        .filename("grim.parquet")
        .build()
        .into_arcx();
    let mnt_ctrl = Mount::new().into_arcx();

    (*cfd_loads.lock().await).stop_after(CFD_DELAY * sim_sampling_frequency);

    let model_1 = {
        let mut source: Initiator<_> = Actor::new(cfd_loads.clone());
        let mut sink = Terminator::<_>::new(logging.clone());
        // FEM
        let mut fem: Actor<_> = Actor::new(state_space.clone());
        // MOUNT
        let mut mount: Actor<_> = Actor::new(mnt_ctrl.clone());

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
                arrow_client::Arrow,
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

        // M1 SYSTEM
        let mut m1_actors = M1System::<M1_RATE, SH48_RATE>::new(n_step, 27)?;

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
        /*         fem.add_output()
                   .bootstrap()
                   .build::<OSSHardpointD>()
                   .into_input(&mut m1_hp_loadcells);
        */
        // M2 POSITIONER COMMAND
        let mut m2_pos_cmd: Initiator<_> = (Signals::new(42, n_step), "M2 Positionners 0pt").into();
        // FSM POSITIONNER
        let mut m2_positionner: Actor<_> =
            (fsm::positionner::Controller::new(), "M2 Positionners").into();
        m2_pos_cmd
            .add_output()
            .build::<M2poscmd>()
            .into_input(&mut m2_positionner);
        m2_positionner
            .add_output()
            .build::<MCM2SmHexF>()
            .into_input(&mut fem);
        // FSM PIEZOSTACK COMMAND
        //let mut m2_pzt_cmd: Initiator<_> = (Signals::new(21, n_step), "M2_PZT_setpoint").into();
        // FSM PIEZOSTACK
        let mut m2_piezostack: Actor<_> =
            (fsm::piezostack::Controller::new(), "M2 PZT Actuators").into();
        /*m2_pzt_cmd
        .add_output()
        .build::< PZTcmd>()
        .into_input(&mut m2_piezostack);*/
        m2_piezostack
            .add_output()
            .build::<MCM2PZTF>()
            .into_input(&mut fem);

        fem.add_output()
            .bootstrap()
            .build::<MCM2SmHexD>()
            .into_input(&mut m2_positionner);
        fem.add_output()
            .bootstrap()
            .build::<MCM2PZTD>()
            .into_input(&mut m2_piezostack);
        // FSM TIP-TILT CONTROL
        let mut tiptilt_set_point: Initiator<_, FSM_RATE> = (
            Into::<Signals>::into((vec![0f64; 14], n_step)),
            "TipTilt_setpoint",
        )
            .into();
        let mut m2_tiptilt: Actor<_, FSM_RATE, 1> =
            (fsm::tiptilt::Controller::new(), "M2 TipTilt Control").into();
        tiptilt_set_point
            .add_output()
            .build::<TTSP>()
            .into_input(&mut m2_tiptilt);
        m2_tiptilt
            .add_output()
            .bootstrap()
            .build::<PZTcmd>()
            .into_input(&mut m2_piezostack);
        // OPTICAL MODEL (SH24)
        println!("SH24");
        let atm_duration = 20f32;
        let atm_n_duration = Some((sim_duration / atm_duration as f64).ceil() as i32);
        let atm_sampling = 48 * 16 + 1;
        let atm = Atmosphere::builder().ray_tracing(
            25.5,
            atm_sampling,
            20f32.from_arcmin(),
            atm_duration,
            Some("/fsx/atmosphere/free_atm_15mn.bin".to_owned()),
            atm_n_duration,
        );
        let tau = (sim_sampling_frequency as f64).recip();
        /*         let free_atm = ceo::OpticalModelOptions::Atmosphere {
                   builder: atm.remove_turbulence_layer(0),
                   time_step: tau,
               };
               let dome_seeing = ceo::OpticalModelOptions::DomeSeeing {
                   cfd_case: "/fsx/CASES/zen30az000_OS7".to_string(),
                   upsampling_rate: (sim_sampling_frequency / 5) as usize,
               };
               let static_aberration = {
                   let gmt_modes_path = std::env::var("GMT_MODES_PATH")?;
                   let path_to_static = Path::new(&gmt_modes_path);
                   let static_phase: Vec<f32> = bincode::deserialize_from(File::open(
                       path_to_static.join("raw-polishing_print-through_soak1deg_769.bin"),
                   )?)?;
                   ceo::OpticalModelOptions::StaticAberration(static_phase.into())
               };
               let gmt_builder = Gmt::builder().m1_n_mode(162);
        */
        let static_phase: Vec<f32> = {
            let gmt_modes_path = std::env::var("GMT_MODES_PATH")?;
            let path_to_static = Path::new(&gmt_modes_path);
            bincode::deserialize_from(File::open(
                path_to_static.join("raw-polishing_print-through_soak1deg_769.bin"),
            )?)?
        };
        let agws = AGWS::new()
            .gmt(Gmt::builder().m1_n_mode(162))
            .atmosphere(atm.remove_turbulence_layer(0), tau)
            .dome_seeing(
                "/fsx/CASES/zen30az000_OS7".to_string(),
                (sim_sampling_frequency / 5) as usize,
            )
            .static_aberration(static_phase);
        let senses: OpticalSensitivities = Loader::<OpticalSensitivities>::default().load()?;
        let rxy_2_stt = senses[OpticalSensitivity::SegmentTipTilt(Vec::new())].m2_rxy()?;
        use calibrations::Mirror;
        use calibrations::Segment::*;
        let sh24 = agws::AgwsShackHartmann::<agws::SH24, agws::Diffractive, FSM_RATE>::builder(1)
            .flux_threshold(0.5)
            .poker(vec![
                Some(vec![(Mirror::M2, vec![Rxyz(1e-6, Some(0..2))])]);
                7
            ])
            .left_pinv(rxy_2_stt);
        let n_sh48 = 1;
        let sh48 =
            agws::AgwsShackHartmann::<agws::SH48, agws::Diffractive, SH48_RATE>::builder(n_sh48)
                .flux_threshold(0.5)
                .poker(vec![
                    Some(vec![(Mirror::M1MODES, vec![Modes(1e-6, 0..27)])]);
                    7
                ]);
        let (mut agws_tt7, mut agws_sh48) = match agws.build(Some(sh24), Some(sh48)) {
            (Some(agws_tt7), Some(agws_sh48)) => (agws_tt7, agws_sh48),
            _ => unimplemented!(),
        };
        agws_tt7
            .add_output()
            .build::<TTFB>()
            .into_input(&mut m2_tiptilt);

        /*         let mut agws_tt7: Actor<_, 1, FSM_RATE> = {
                   let mut agws_sh24 = ceo::OpticalModel::builder()
                       .gmt(gmt_builder.clone())
                       .source(Source::builder())
                       .options(vec![
                           ceo::OpticalModelOptions::ShackHartmann {
                               options: ceo::ShackHartmannOptions::Diffractive(
                                   *TT7::<crseo::Diffractive>::new(),
                               ),
                               flux_threshold: 0.5,
                           },
                           free_atm.clone(),
                           dome_seeing.clone(),
                           static_aberration.clone(),
                       ])
                       .build()?;
                   use calibrations::Mirror;
                   use calibrations::Segment::*;
                   // GMT 2 WFS
                   println!(" - calibration ...");
                   let mut gmt2wfs = Calibration::new(
                       &agws_sh24.gmt,
                       &agws_sh24.src,
                       TT7::<crseo::Geometric>::new(),
                   );
                   let specs = vec![Some(vec![(Mirror::M2, vec![Rxyz(1e-6, Some(0..2))])]); 7];
                   let now = Instant::now();
                   gmt2wfs.calibrate(
                       specs,
                       calibrations::ValidLensletCriteria::OtherSensor(
                           &mut agws_sh24.sensor.as_mut().unwrap(),
                       ),
                   );
                   println!(
                       "GMT 2 WFS calibration [{}x{}] in {}s",
                       gmt2wfs.n_data,
                       gmt2wfs.n_mode,
                       now.elapsed().as_secs()
                   );
                   let dof_2_wfs: Vec<f64> = gmt2wfs.poke.into();
                   let dof_2_wfs = na::DMatrix::<f64>::from_column_slice(
                       dof_2_wfs.len() / gmt2wfs.n_mode,
                       gmt2wfs.n_mode,
                       &dof_2_wfs,
                   );
                   let wfs_2_rxy = dof_2_wfs.clone().pseudo_inverse(1e-12).unwrap();
                   let senses: OpticalSensitivities = Loader::<OpticalSensitivities>::default().load()?;
                   let rxy_2_stt = senses[OpticalSensitivity::SegmentTipTilt(Vec::new())].m2_rxy()?;
                   agws_sh24.sensor_matrix_transform(rxy_2_stt * wfs_2_rxy);
                   (agws_sh24, "AGWS SH24").into()
               };
               agws_tt7
                   .add_output()
                   .build::<TTFB>()
                   .into_input(&mut m2_tiptilt);

               // OPTICAL MODEL (SH48)
               println!("SH48");
               let n_sh48 = 1;
               let gmt_agws_sh48 = {
                   let mut agws_sh48 = ceo::OpticalModel::builder()
                       .gmt(gmt_builder)
                       .source(Source::builder().on_ring(6f32.from_arcmin()))
                       .options(vec![
                           ceo::OpticalModelOptions::ShackHartmann {
                               options: ceo::ShackHartmannOptions::Diffractive(
                                   *SH48::<crseo::Diffractive>::new().n_sensor(n_sh48),
                               ),
                               flux_threshold: 0.5,
                           },
                           free_atm,
                           dome_seeing,
                           static_aberration,
                       ])
                       .build()?;
                   let data_repo = env::var("DATA_REPO").unwrap_or_else(|_| ".".to_string());
                   let filename = format!("sh48x{}-diff_2_m1-modes.bin", n_sh48);
                   let poke_mat_file = Path::new(&data_repo).join(&filename);
                   let wfs_2_dof: na::DMatrix<f64> = if poke_mat_file.is_file() {
                       println!(" . Poke matrix loaded from {poke_mat_file:?}");
                       let file = File::open(poke_mat_file)?;
                       bincode::deserialize_from(file)?
                   } else {
                       println!(" - calibration ...");
                       use calibrations::Mirror;
                       use calibrations::Segment::*;
                       // GMT 2 WFS
                       let mut gmt2sh48 = Calibration::new(
                           &agws_sh48.gmt,
                           &agws_sh48.src,
                           SH48::<crseo::Geometric>::new().n_sensor(n_sh48),
                       );
                       let specs = vec![Some(vec![(Mirror::M1MODES, vec![Modes(1e-6, 0..27)])]); 7];
                       let now = Instant::now();
                       gmt2sh48.calibrate(
                           specs,
                           calibrations::ValidLensletCriteria::OtherSensor(
                               &mut agws_sh48.sensor.as_mut().unwrap(),
                           ),
                       );
                       println!(
                           "GMT 2 SH48 calibration [{}x{}] in {}s",
                           gmt2sh48.n_data,
                           gmt2sh48.n_mode,
                           now.elapsed().as_secs()
                       );
                       let dof_2_wfs: Vec<f64> = gmt2sh48.poke.into();
                       let dof_2_wfs = na::DMatrix::<f64>::from_column_slice(
                           dof_2_wfs.len() / gmt2sh48.n_mode,
                           gmt2sh48.n_mode,
                           &dof_2_wfs,
                       );
                       let singular_values = dof_2_wfs.singular_values();
                       let max_sv: f64 = singular_values[0];
                       let min_sv: f64 = singular_values.as_slice().iter().last().unwrap().clone();
                       let condition_number = max_sv / min_sv;
                       println!("SH48 poke matrix condition number: {condition_number:e}");
                       let wfs_2_dof = dof_2_wfs.clone().pseudo_inverse(1e-12).unwrap();
                       let mut file = File::create(&poke_mat_file)?;
                       bincode::serialize_into(&mut file, &wfs_2_dof)?;
                       println!(" . Poke matrix saved to {poke_mat_file:?}");
                       wfs_2_dof
                   };
                   agws_sh48.sensor_matrix_transform(wfs_2_dof);
                   agws_sh48.into_arcx()
               };
               let name = format!("AGWS SH48 (x{})", n_sh48);
               let mut agws_sh48: Actor<_, 1, SH48_RATE> = Actor::new(gmt_agws_sh48.clone()).name(name);
        */
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

        /*
        let mut mode_m1s = vec![
            {
                let mut mode = vec![0f64; 162];
                mode[0] = 1e-6;
                na::DVector::from_vec(mode)
            };
            6
        ];
        //mode_m1s[0][0] = 1e-6;
        mode_m1s.push({
            let mut mode = vec![0f64; 151];
            mode[0] = 1e-6;
            na::DVector::from_vec(mode)
        });
        let zero_point: Vec<_> = mode_m1s
            .iter()
            .flat_map(|x| x.as_slice().to_vec())
            .collect();
         */
        let zero_point = vec![0f64; 27 * 7];
        /*
        zero_point.chunks_mut(27).for_each(|x| {
                x[0] = 1e-6;
                x[26] = 2e-7;
            });
        */
        /*
            println!("{zero_point:#?}");
            let mut m1_modes: Initiator<_, SH48_RATE> = Into::<Signals>::into((zero_point, n_step)).into();
                //dbg!(&zero_point);
        */
        let mut gain = vec![0.; 7 * 27];
        gain.iter_mut().skip(26).step_by(27).for_each(|g| *g = 0.5);
        let mut integrator: Actor<_, SH48_RATE, SH48_RATE> =
            Integrator::<f64, ceo::SensorData>::new(27 * 7)
                //.gain_vector(gain)
                .gain(0.5)
                .zero(zero_point)
                .into();
        let sh48_arrow = Arrow::builder(n_step).filename("sh48.parquet").build();
        let mut sh48_log: Terminator<_, SH48_RATE> = (sh48_arrow, "SH48_Log").into();

        agws_sh48
            .add_output()
            .multiplex(2)
            .build::<ceo::SensorData>()
            .into_input(&mut integrator)
            .logn(&mut sh48_log, 27 * 7)
            .await
            .confirm()?
            .add_output()
            .build::<ceo::WfeRms>()
            .log(&mut sh48_log)
            .await
            .confirm()?
            .add_output()
            .build::<ceo::DetectorFrame>()
            .logn(&mut sh48_log, 48 * 48 * 8 * 8 * n_sh48)
            .await;

        let sh24_arrow = Arrow::builder(n_step)
            .filename("sh24.parquet")
            //.decimation(10)
            .build();
        let mut sh24_log: Terminator<_, FSM_RATE> = (sh24_arrow, "SH24_Log").into();

        agws_tt7
            .add_output()
            .build::<ceo::WfeRms>()
            .log(&mut sh24_log)
            .await;
        agws_tt7
            .add_output()
            .build::<ceo::TipTilt>()
            .log(&mut sh24_log)
            .await;
        agws_tt7
            .add_output()
            .build::<ceo::SegmentWfeRms>()
            .log(&mut sh24_log)
            .await;
        agws_tt7
            .add_output()
            .build::<ceo::SegmentPiston>()
            .log(&mut sh24_log)
            .await;
        agws_tt7
            .add_output()
            .build::<ceo::SegmentTipTilt>()
            .log(&mut sh24_log)
            .await;

        #[derive(UID)]
        #[uid(data = "Vec<f32>")]
        enum SH24Frame {}
        let mut sh24_frame_sampler: Actor<_, FSM_RATE, { FSM_RATE * 200 }> = (
            Sampler::<Vec<f32>, ceo::DetectorFrame, SH24Frame>::default(),
            "SH24 Frame",
        )
            .into();
        agws_tt7
            .add_output()
            .build::<ceo::DetectorFrame>()
            .into_input(&mut &mut sh24_frame_sampler);
        let mut sh24_frame_logger: Terminator<_, { FSM_RATE * 200 }> = (
            Arrow::builder(n_step)
                .filename("sh24-frame.parquet")
                .build(),
            "SH24 Frame Logs",
        )
            .into();
        sh24_frame_sampler
            .add_output()
            .build::<SH24Frame>()
            .logn(&mut sh24_frame_logger, 24 * 24 * 12 * 12)
            .await;

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
        let mut actors: Vec<Box<dyn Task>> = vec![
            Box::new(source),
            Box::new(mount_set_point),
            Box::new(mount),
            Box::new(m2_pos_cmd),
            Box::new(m2_positionner),
            Box::new(m2_piezostack),
            Box::new(tiptilt_set_point),
            Box::new(m2_tiptilt),
            Box::new(agws_tt7),
            Box::new(agws_sh48),
            Box::new(integrator),
            Box::new(sh48_log),
            Box::new(sh24_log),
            Box::new(sh24_frame_sampler),
            Box::new(sh24_frame_logger),
            Box::new(fem),
            Box::new(sink),
        ];
        actors.append(&mut m1_actors);
        let model = Model::new(actors).name("im-fsm").flowchart().check()?.run();

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

        /*         let sh48_progress = progress.clone();
               tokio::spawn(async move {
                   let mut interval = tokio::time::interval(Duration::from_secs(1));
                   let bar: Bar = sh48_progress
                       .lock()
                       .await
                       .bar(SH48_RATE, "SH48 integration");
                   loop {
                       interval.tick().await;
                       let mut progress = sh48_progress.lock().await;
                       progress.set_and_draw(
                           &bar,
                           (*gmt_agws_sh48.lock().await)
                               .sensor
                               .as_ref()
                               .unwrap()
                               .n_frame(),
                       );
                   }
               });
        */
        model.wait().await?;
    }

    /*
    let lom = LOM::builder()
        .rigid_body_motions_record((*logging.lock().await).record()?)?
        .build()?;
    let segment_tiptilt = lom.segment_tiptilt();

    let tau = (sim_sampling_frequency as f64).recip();
    let _: complot::Plot = ((
        segment_tiptilt
            .items()
            .enumerate()
            .map(|(i, data)| (i as f64 * tau, data.to_owned().to_mas())),
        None,
    ))
        .into();
    */
    Ok(())
}
