use core::fmt;
use std::{collections::HashMap, sync::Arc};

use crseo::{Atmosphere, FromBuilder};
use dos_actors::{
    clients::{
        arrow_client::Arrow,
        ceo::{
            DetectorFrame, M1rbm, M2rbm, OpticalModel, OpticalModelOptions, PSSnFwhm, PSSnOptions,
            SegmentTipTilt, SensorData, ShackHartmannOptions,
        },
        Integrator,
    },
    io::{Data, Read, Write},
    prelude::*,
    Update,
};
use grim::{agws, control};
use skyangle::Conversion;

#[derive(UID)]
pub enum Rxy {}

pub struct Rxy2RBM {
    rxy: Arc<Data<Rxy>>,
}

impl Rxy2RBM {
    pub fn new() -> Self {
        Self {
            rxy: Arc::new(Data::new(vec![])),
        }
    }
}
impl Update for Rxy2RBM {}
impl Read<Vec<f64>, Rxy> for Rxy2RBM {
    fn read(&mut self, data: Arc<Data<Rxy>>) {
        self.rxy = data.clone();
    }
}
impl Write<Vec<f64>, M2rbm> for Rxy2RBM {
    fn write(&mut self) -> Option<Arc<Data<M2rbm>>> {
        let data: &[f64] = &self.rxy;
        let rbm: Vec<f64> = data
            .chunks(2)
            .flat_map(|x| vec![0., 0., 0., x[0], x[1], 0.])
            .collect();
        Some(Arc::new(Data::new(rbm)))
    }
}

enum SignalType {
    Sinus,
    Const,
}

#[derive(Default)]
pub struct Alias<T: Default> {
    data: T,
}
impl<T: Default> Update for Alias<T> {}
impl<T, U> Write<T, U> for Alias<T>
where
    T: Default + Clone,
    U: UniqueIdentifier<Data = T>,
{
    fn write(&mut self) -> Option<Arc<Data<U>>> {
        Some(Arc::new(Data::new(self.data.clone())))
    }
}
impl<T, U> Read<T, U> for Alias<T>
where
    T: Default + ToOwned<Owned = T>,
    U: UniqueIdentifier<Data = T>,
{
    fn read(&mut self, data: Arc<Data<U>>) {
        let this_data: &T = &data;
        self.data = this_data.to_owned();
    }
}

#[derive(UID)]
pub enum ClosedLoopSegmentTipTilt {}
#[derive(UID)]
#[uid(data = "Vec<f32>")]
pub enum ClosedLoopDetectorFrame {}
#[derive(UID)]
pub enum ClosedLoopPSSnFwhm {}

pub struct Fsm {
    data: Vec<f64>,
    dynamics: control::M2Dynamics,
}

impl Fsm {
    pub fn new() -> Self {
        Self {
            data: Vec::new(),
            dynamics: control::M2Dynamics::fsm(14),
        }
    }
}
impl Update for Fsm {}
impl Read<Vec<f64>, Rxy> for Fsm {
    fn read(&mut self, data: Arc<Data<Rxy>>) {
        let inner: &[f64] = &data;
        self.data = inner.to_vec();
    }
}
impl Write<Vec<f64>, M2rbm> for Fsm {
    fn write(&mut self) -> Option<Arc<Data<M2rbm>>> {
        self.dynamics
            .step(self.data.as_slice())
            .map(|x| {
                let rbm: Vec<f64> = x
                    .chunks(2)
                    .flat_map(|x| vec![0., 0., 0., x[0], x[1], 0.])
                    .collect();
                rbm
            })
            .map(|x| Arc::new(Data::new(x)))
    }
}

#[derive(Debug, Clone)]
pub enum SH24GuideStar {
    OnAxis,
    OffAxis(f32),
}
impl From<SH24GuideStar> for Vec<f32> {
    fn from(gs: SH24GuideStar) -> Self {
        vec![match gs {
            SH24GuideStar::OnAxis => 0f32,
            SH24GuideStar::OffAxis(value) => value,
        }]
    }
}
impl From<SH24GuideStar> for HashMap<String, String> {
    fn from(gs: SH24GuideStar) -> Self {
        let zen_arcmin = Into::<Vec<f32>>::into(gs)[0].to_arcmin();
        let mut map = HashMap::new();
        map.insert(
            "off-axis distance [arcmin]".to_string(),
            format!("{zen_arcmin:.2}"),
        );
        map
    }
}
impl fmt::Display for SH24GuideStar {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SH24GuideStar::OnAxis => write!(f, "OnAxis"),
            SH24GuideStar::OffAxis(value) => write!(f, "{:.0}ArcminOffAxis", value.to_arcmin()),
        }
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    std::env::set_var(
        "DATA_REPO",
        std::path::Path::new(&std::env::var("CARGO_MANIFEST_DIR")?)
            .join("src")
            .join("bin")
            .join("tiptilt"),
    );

    let duration = 30_000;
    let sampling_frequency = 1000f64;
    println!(
        "simulation duration {:.0}s",
        duration as f64 * &sampling_frequency
    );

    //let sh24_guide_star = SH24GuideStar::OffAxis(6f32.from_arcmin());
    let sh24_guide_star = SH24GuideStar::OnAxis;
    println!("SH24: {:}", sh24_guide_star);

    let atm_duration = 20f32;
    let atm_n_duration = Some((910. / atm_duration as f64).ceil() as i32);
    let atm_sampling = 48 * 16 + 1;
    let atm = Atmosphere::builder().ray_tracing(
        25.5,
        atm_sampling,
        20f32.from_arcmin(),
        atm_duration,
        Some("/fsx/atmosphere/atm_15mn.bin".to_owned()),
        atm_n_duration,
    );

    let imgr = OpticalModelOptions::ShackHartmann {
        options: ShackHartmannOptions::Diffractive(
            crseo::ShackHartmann::<crseo::Diffractive>::builder(),
        ),
        flux_threshold: 0f64,
    };

    let pssn = OpticalModelOptions::PSSn(PSSnOptions::AtmosphereTelescope(crseo::PSSn::builder()));

    let options = vec![
        imgr,
        OpticalModelOptions::Atmosphere {
            builder: atm.clone(),
            time_step: sampling_frequency.recip(),
        },
        pssn,
    ];
    let closed_loop_optical_model = OpticalModel::builder()
        .options(options.clone())
        .build()?
        .into_arcx();
    let open_loop_optical_model = OpticalModel::builder()
        .options(options)
        .build()?
        .into_arcx();

    {
        let mut closed_loop: Actor<_> = Actor::new(closed_loop_optical_model.clone()).name(
            r#"On-axis closed-loop
    w/ atmosphere"#,
        );
        let mut open_loop: Actor<_> = Actor::new(open_loop_optical_model.clone()).name(
            r#"On-axis open-loop
    w/ atmosphere"#,
        );

        let mut sh24: Actor<_, 1, 5> = agws::AGWS::new()
            .atmosphere(atm.clone(), sampling_frequency.recip())
            .build(
                Some(
                    agws::AgwsShackHartmann::<agws::SH24, agws::Geometric, 5>::builder(1)
                        .guide_star(
                            crseo::Source::builder()
                                .zenith_azimuth(sh24_guide_star.clone().into(), vec![0f32]),
                        )
                        .with_m2_tiptilt(),
                ),
                Option::<agws::AgwsShackHartmann<agws::SH48, agws::Geometric, 5>>::None,
            )
            .into();

        let mut signal: Initiator<_> = match SignalType::Const {
            SignalType::Sinus => (0..7)
                .fold(Signals::new(42, duration), |s, i| {
                    let sin = Signal::Sinusoid {
                        amplitude: 100f64.from_mas(),
                        sampling_frequency_hz: 1000.,
                        frequency_hz: 1f64 / (i + 1) as f64,
                        phase_s: 0f64,
                    };
                    let ix = 3 + i * 6;
                    s.output_signal(ix, sin.clone())
                        .output_signal(ix + 1, sin.clone())
                })
                .into(),
            SignalType::Const => Signals::new(42, duration).into(),
        };

        let mut logs: Terminator<_> = (
            Arrow::builder(duration)
                .filename(format!("data-{:}", sh24_guide_star))
                .metadata(sh24_guide_star.clone().into())
                .build(),
            "Logs",
        )
            .into();

        signal
            .add_output()
            .multiplex(3)
            .build::<M1rbm>()
            .into_input(&mut sh24)
            .into_input(&mut open_loop)
            .into_input(&mut closed_loop)
            .confirm()?;
        open_loop
            .add_output()
            .unbounded()
            .build::<SegmentTipTilt>()
            .log(&mut logs)
            .await;

        let mut alias: Actor<_> = Alias::<Vec<f64>>::default().into();
        closed_loop
            .add_output()
            .unbounded()
            .build::<SegmentTipTilt>()
            .into_input(&mut alias);
        alias
            .add_output()
            .build::<ClosedLoopSegmentTipTilt>()
            .logn(&mut logs, 14)
            .await;

        let mut integrator: Actor<_, 5, 1> =
            Integrator::<f64, SensorData>::new(14).gain(0.3).into();
        let mut fsm: Actor<_> = (Fsm::new(), "FSM").into();
        //let mut rxy2rbm: Actor<_, 1, 1> = Rxy2RBM::new().into();

        sh24.add_output()
            .build::<SensorData>()
            .into_input(&mut integrator);
        integrator
            .add_output()
            .bootstrap()
            .build::<Rxy>()
            .into_input(&mut fsm);
        fsm.add_output()
            .multiplex(2)
            .build::<M2rbm>()
            .into_input(&mut sh24)
            .into_input(&mut closed_loop)
            .confirm()?;

        Model::new(vec![
            Box::new(open_loop),
            Box::new(closed_loop),
            Box::new(signal),
            Box::new(alias),
            Box::new(logs),
            Box::new(sh24),
            Box::new(integrator),
            Box::new(fsm),
        ])
        .name("tiptilt-feedback-loop")
        .check()?
        .flowchart()
        .run()
        .wait()
        .await?;
    }

    {
        let mut closed_loop: Actor<_> = Actor::new(closed_loop_optical_model.clone()).name(
            r#"On-axis closed-loop
    w/ atmosphere"#,
        );
        let mut open_loop: Actor<_> = Actor::new(open_loop_optical_model.clone()).name(
            r#"On-axis open-loop
    w/ atmosphere"#,
        );
        let mut frame_logs: Terminator<_> = (
            Arrow::builder(1)
                .metadata(sh24_guide_star.clone().into())
                .filename(format!("frames-{:}", sh24_guide_star))
                .build(),
            "Detector Frames",
        )
            .into();
        let mut timer: Initiator<_> = Timer::new(0).into();

        timer
            .add_output()
            .multiplex(2)
            .build::<Tick>()
            .into_input(&mut open_loop)
            .into_input(&mut closed_loop)
            .confirm()?;
        open_loop
            .add_output()
            .bootstrap()
            .build::<DetectorFrame>()
            .logn(&mut frame_logs, 512 * 512)
            .await;
        open_loop
            .add_output()
            .bootstrap()
            .build::<PSSnFwhm>()
            .logn(&mut frame_logs, 2)
            .await;
        let mut frame_alias: Actor<_> = Alias::<Vec<f32>>::default().into();
        closed_loop
            .add_output()
            .bootstrap()
            .build::<DetectorFrame>()
            .into_input(&mut frame_alias);
        frame_alias
            .add_output()
            .build::<ClosedLoopDetectorFrame>()
            .logn(&mut frame_logs, 512 * 512)
            .await;
        let mut pssn_alias: Actor<_> = Alias::default().into();
        closed_loop
            .add_output()
            .bootstrap()
            .build::<PSSnFwhm>()
            .into_input(&mut pssn_alias);
        pssn_alias
            .add_output()
            .build::<ClosedLoopPSSnFwhm>()
            .logn(&mut frame_logs, 2)
            .await;

        Model::new(vec![
            Box::new(timer),
            Box::new(open_loop),
            Box::new(closed_loop),
            Box::new(frame_alias),
            Box::new(pssn_alias),
            Box::new(frame_logs),
        ])
        .name("framer")
        .check()?
        .flowchart()
        .run()
        .wait()
        .await?;
    }

    Ok(())
}
