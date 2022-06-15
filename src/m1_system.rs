//! # GMT M1 system

use std::sync::Arc;

use crate::{Error, Result};
use dos_actors::{
    clients::{
        m1::{
            M1RBMcmd, Mode2Force, S1SAoffsetFcmd, S2SAoffsetFcmd, S3SAoffsetFcmd, S4SAoffsetFcmd,
            S5SAoffsetFcmd, S6SAoffsetFcmd, S7SAoffsetFcmd, S1HPLC, S2HPLC, S3HPLC, S4HPLC, S5HPLC,
            S6HPLC, S7HPLC,
        },
        Signals,
    },
    io::{Data, Write},
    prelude::UniqueIdentifier,
    Actor, AddOuput, Initiator, IntoInputs, Task, Update,
};
use fem::{
    dos::{DiscreteModalSolver, ExponentialMatrix},
    fem_io::{
        M1ActuatorsSegment1, M1ActuatorsSegment2, M1ActuatorsSegment3, M1ActuatorsSegment4,
        M1ActuatorsSegment5, M1ActuatorsSegment6, M1ActuatorsSegment7, OSSHardpointD,
        OSSHarpointDeltaF,
    },
};
use m1_ctrl::{
    actuators::{segment1, segment2, segment3, segment4, segment5, segment6, segment7},
    hp_dynamics, hp_load_cells,
};

const U: usize = 1;

/// M1 segment actuators related actors
///
/// Each segment has 2 components:
///  - the actuators control system actor: `actuators` with input rate `R` and output rate `1`
///  - the modal coefficients to actuator forces converter: `modes_2_forces` with input rate `O` and output rate `R`
pub enum Segment<const R: usize, const O: usize = R> {
    S1 {
        actuators: Actor<segment1::Controller<'static>, R, U>,
        modes_2_forces: Actor<Mode2Force<1>, O, R>,
    },
    S2 {
        actuators: Actor<segment2::Controller<'static>, R, U>,
        modes_2_forces: Actor<Mode2Force<2>, O, R>,
    },
    S3 {
        actuators: Actor<segment3::Controller<'static>, R, U>,
        modes_2_forces: Actor<Mode2Force<3>, O, R>,
    },
    S4 {
        actuators: Actor<segment4::Controller<'static>, R, U>,
        modes_2_forces: Actor<Mode2Force<4>, O, R>,
    },
    S5 {
        actuators: Actor<segment5::Controller<'static>, R, U>,
        modes_2_forces: Actor<Mode2Force<5>, O, R>,
    },
    S6 {
        actuators: Actor<segment6::Controller<'static>, R, U>,
        modes_2_forces: Actor<Mode2Force<6>, O, R>,
    },
    S7 {
        actuators: Actor<segment7::Controller<'static>, R, U>,
        modes_2_forces: Actor<Mode2Force<7>, O, R>,
    },
}

impl<const R: usize, const O: usize> Segment<R, O> {
    /// Creates M1 segment #`sid` with `n_mode` *controlled* bending modes
    pub fn new(sid: usize, n_mode: usize) -> Result<Self> {
        match sid {
            sid if sid == 1 => Ok(Segment::S1 {
                actuators: segment1::Controller::new().into(),
                modes_2_forces: (
                    Mode2Force::new(335, 162, format!("m1s{sid}mode2forces.bin"))?
                        .n_input_mode(n_mode),
                    format!("M1S{sid}_M2F"),
                )
                    .into(),
            }),
            sid if sid == 2 => Ok(Segment::S2 {
                actuators: segment2::Controller::new().into(),
                modes_2_forces: (
                    Mode2Force::new(335, 162, format!("m1s{sid}mode2forces.bin"))?
                        .n_input_mode(n_mode),
                    format!("M1S{sid}_M2F"),
                )
                    .into(),
            }),
            sid if sid == 3 => Ok(Segment::S3 {
                actuators: segment3::Controller::new().into(),
                modes_2_forces: (
                    Mode2Force::new(335, 162, format!("m1s{sid}mode2forces.bin"))?
                        .n_input_mode(n_mode),
                    format!("M1S{sid}_M2F"),
                )
                    .into(),
            }),
            sid if sid == 4 => Ok(Segment::S4 {
                actuators: segment4::Controller::new().into(),
                modes_2_forces: (
                    Mode2Force::new(335, 162, format!("m1s{sid}mode2forces.bin"))?
                        .n_input_mode(n_mode),
                    format!("M1S{sid}_M2F"),
                )
                    .into(),
            }),
            sid if sid == 5 => Ok(Segment::S5 {
                actuators: segment5::Controller::new().into(),
                modes_2_forces: (
                    Mode2Force::new(335, 162, format!("m1s{sid}mode2forces.bin"))?
                        .n_input_mode(n_mode),
                    format!("M1S{sid}_M2F"),
                )
                    .into(),
            }),
            sid if sid == 6 => Ok(Segment::S6 {
                actuators: segment6::Controller::new().into(),
                modes_2_forces: (
                    Mode2Force::new(335, 162, format!("m1s{sid}mode2forces.bin"))?
                        .n_input_mode(n_mode),
                    format!("M1S{sid}_M2F"),
                )
                    .into(),
            }),
            sid if sid == 7 => Ok(Segment::S7 {
                actuators: segment7::Controller::new().into(),
                modes_2_forces: (
                    Mode2Force::new(306, 151, format!("m1s{sid}mode2forces.bin"))?
                        .n_input_mode(n_mode),
                    format!("M1S{sid}_M2F"),
                )
                    .into(),
            }),
            _ => Err(Error::Segment),
        }
    }
    /// Connects segment actors inputs and outputs to the `loadcells` and the [fem](fem) and returns the segment actors
    pub fn build(
        self,
        loadcells: &mut Actor<hp_load_cells::Controller<'static>, U, R>,
        fem: &mut Actor<DiscreteModalSolver<ExponentialMatrix>>,
    ) -> Vec<Box<dyn Task>> {
        match self {
            Self::S1 {
                mut actuators,
                mut modes_2_forces,
            } => {
                loadcells
                    .add_output()
                    .bootstrap()
                    .build::<S1HPLC>()
                    .into_input(&mut actuators);
                modes_2_forces
                    .add_output()
                    .build::<S1SAoffsetFcmd>()
                    .into_input(&mut actuators);
                actuators
                    .add_output()
                    .build::<M1ActuatorsSegment1>()
                    .into_input(fem);
                vec![Box::new(actuators), Box::new(modes_2_forces)]
            }
            Self::S2 {
                mut actuators,
                mut modes_2_forces,
            } => {
                loadcells
                    .add_output()
                    .bootstrap()
                    .build::<S2HPLC>()
                    .into_input(&mut actuators);
                modes_2_forces
                    .add_output()
                    .build::<S2SAoffsetFcmd>()
                    .into_input(&mut actuators);
                actuators
                    .add_output()
                    .build::<M1ActuatorsSegment2>()
                    .into_input(fem);
                vec![Box::new(actuators), Box::new(modes_2_forces)]
            }
            Self::S3 {
                mut actuators,
                mut modes_2_forces,
            } => {
                loadcells
                    .add_output()
                    .bootstrap()
                    .build::<S3HPLC>()
                    .into_input(&mut actuators);
                modes_2_forces
                    .add_output()
                    .build::<S3SAoffsetFcmd>()
                    .into_input(&mut actuators);
                actuators
                    .add_output()
                    .build::<M1ActuatorsSegment3>()
                    .into_input(fem);
                vec![Box::new(actuators), Box::new(modes_2_forces)]
            }
            Self::S4 {
                mut actuators,
                mut modes_2_forces,
            } => {
                loadcells
                    .add_output()
                    .bootstrap()
                    .build::<S4HPLC>()
                    .into_input(&mut actuators);
                modes_2_forces
                    .add_output()
                    .build::<S4SAoffsetFcmd>()
                    .into_input(&mut actuators);
                actuators
                    .add_output()
                    .build::<M1ActuatorsSegment4>()
                    .into_input(fem);
                vec![Box::new(actuators), Box::new(modes_2_forces)]
            }
            Self::S5 {
                mut actuators,
                mut modes_2_forces,
            } => {
                loadcells
                    .add_output()
                    .bootstrap()
                    .build::<S5HPLC>()
                    .into_input(&mut actuators);
                modes_2_forces
                    .add_output()
                    .build::<S5SAoffsetFcmd>()
                    .into_input(&mut actuators);
                actuators
                    .add_output()
                    .build::<M1ActuatorsSegment5>()
                    .into_input(fem);
                vec![Box::new(actuators), Box::new(modes_2_forces)]
            }
            Self::S6 {
                mut actuators,
                mut modes_2_forces,
            } => {
                loadcells
                    .add_output()
                    .bootstrap()
                    .build::<S6HPLC>()
                    .into_input(&mut actuators);
                modes_2_forces
                    .add_output()
                    .build::<S6SAoffsetFcmd>()
                    .into_input(&mut actuators);
                actuators
                    .add_output()
                    .build::<M1ActuatorsSegment6>()
                    .into_input(fem);
                vec![Box::new(actuators), Box::new(modes_2_forces)]
            }
            Self::S7 {
                mut actuators,
                mut modes_2_forces,
            } => {
                loadcells
                    .add_output()
                    .bootstrap()
                    .build::<S7HPLC>()
                    .into_input(&mut actuators);
                modes_2_forces
                    .add_output()
                    .build::<S7SAoffsetFcmd>()
                    .into_input(&mut actuators);
                actuators
                    .add_output()
                    .build::<M1ActuatorsSegment7>()
                    .into_input(fem);
                vec![Box::new(actuators), Box::new(modes_2_forces)]
            }
        }
    }
    /// Connects modal coefficients to `modes_2_force` inputs
    pub fn add_input<'a, CO, DU, const NI: usize>(
        &mut self,
        output_actor_rx: (
            &'a mut Actor<CO, NI, O>,
            Vec<flume::Receiver<Arc<Data<DU>>>>,
        ),
    ) -> (
        &'a mut Actor<CO, NI, O>,
        Vec<flume::Receiver<Arc<Data<DU>>>>,
    )
    where
        DU: 'static + UniqueIdentifier<Data = Vec<f64>>,
        CO: 'static + Update + Send + Write<Vec<f64>, DU>,
        Mode2Force<1_usize>: dos_actors::io::Read<Vec<f64>, DU>,
        Mode2Force<2_usize>: dos_actors::io::Read<Vec<f64>, DU>,
        Mode2Force<3_usize>: dos_actors::io::Read<Vec<f64>, DU>,
        Mode2Force<4_usize>: dos_actors::io::Read<Vec<f64>, DU>,
        Mode2Force<5_usize>: dos_actors::io::Read<Vec<f64>, DU>,
        Mode2Force<6_usize>: dos_actors::io::Read<Vec<f64>, DU>,
        Mode2Force<7_usize>: dos_actors::io::Read<Vec<f64>, DU>,
    {
        match self {
            Self::S1 {
                ref mut modes_2_forces,
                ..
            } => output_actor_rx.into_input(modes_2_forces),
            Self::S2 {
                ref mut modes_2_forces,
                ..
            } => output_actor_rx.into_input(modes_2_forces),
            Self::S3 {
                ref mut modes_2_forces,
                ..
            } => output_actor_rx.into_input(modes_2_forces),
            Self::S4 {
                ref mut modes_2_forces,
                ..
            } => output_actor_rx.into_input(modes_2_forces),
            Self::S5 {
                ref mut modes_2_forces,
                ..
            } => output_actor_rx.into_input(modes_2_forces),
            Self::S6 {
                ref mut modes_2_forces,
                ..
            } => output_actor_rx.into_input(modes_2_forces),
            Self::S7 {
                ref mut modes_2_forces,
                ..
            } => output_actor_rx.into_input(modes_2_forces),
        }
    }
}

/// M1 system actors
///
/// The M1 system actors are
/// - M1 rigid body motions set points: `rbm_set_point`
/// - M1 hardoints: `hardpoints`
/// - M1 hardpoints load cells: `hp_loadcells`
/// - M1 segments: [segments](Segment)
pub struct M1System<const R: usize, const O: usize> {
    rbm_set_point: Initiator<Signals>,
    hardpoints: Actor<hp_dynamics::Controller<'static>>,
    hp_loadcells: Actor<hp_load_cells::Controller<'static>, U, R>,
    segments: Vec<Segment<R, O>>,
}

impl<const R: usize, const O: usize> M1System<R, O> {
    /// Creates M1 system with `n_step` time step and `n_mode` *controlled* bending modes
    pub fn new(n_step: usize, n_mode: usize) -> Result<Self> {
        Ok(Self {
            rbm_set_point: (Signals::new(42, n_step), "M1 RBM 0pt").into(),
            hardpoints: (hp_dynamics::Controller::new(), "M1 Hardpoints").into(),
            hp_loadcells: (hp_load_cells::Controller::new(), "M1 LoadCells").into(),
            segments: (1..=7)
                .map(|sid| Segment::new(sid, n_mode))
                .collect::<Result<Vec<Segment<R, O>>>>()?,
        })
    }
    /// Return a mutable iterator over the segments
    pub fn segments(&mut self) -> impl Iterator<Item = &mut Segment<R, O>> {
        self.segments.iter_mut()
    }
    /// Connects inputs and outputs of M1 system actors adn return the vector of actors
    pub fn build(
        mut self,
        fem: &mut Actor<DiscreteModalSolver<ExponentialMatrix>>,
    ) -> Vec<Box<dyn Task>> {
        self.rbm_set_point
            .add_output()
            .build::<M1RBMcmd>()
            .into_input(&mut self.hardpoints);
        self.hardpoints
            .add_output()
            .multiplex(2)
            .build::<OSSHarpointDeltaF>()
            .into_input(fem)
            .into_input(&mut self.hp_loadcells);
        fem.add_output()
            .bootstrap()
            .build::<OSSHardpointD>()
            .into_input(&mut self.hp_loadcells);
        let segments_actors: Vec<Box<dyn Task>> = self
            .segments
            .into_iter()
            .flat_map(|segment| segment.build(&mut self.hp_loadcells, fem))
            .collect();
        vec![
            Box::new(self.rbm_set_point) as Box<dyn Task>,
            Box::new(self.hardpoints) as Box<dyn Task>,
            Box::new(self.hp_loadcells) as Box<dyn Task>,
        ]
        .into_iter()
        .chain(segments_actors)
        .collect()
    }
}
