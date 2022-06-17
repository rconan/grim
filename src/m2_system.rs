//! # GMT M2 system

use dos_actors::{
    clients::{
        fsm::{M2poscmd, PZTcmd, TTSP},
        Signals,
    },
    Actor, AddOuput, Initiator, IntoInputs, Task,
};
use fem::{
    dos::{DiscreteModalSolver, ExponentialMatrix},
    fem_io::{MCM2SmHexD, MCM2SmHexF, MCM2PZTD, MCM2PZTF},
};
use fsm::{piezostack, positionner, tiptilt};

/// M2 system actors
///
/// The M2 system actors are
/// - FSM positionner set point: `positionner_set_point`
/// - FSM positionner: `positionner`
/// - FSM piezostack actuators: `piezostack`
/// - FSM tip-tilt set point: `tiptilt_set_point`
/// - FSM tip-tilt controller: `tiptilt_`
pub struct M2System<const FSM_RATE: usize> {
    positionner_set_point: Initiator<Signals>,
    positionner: Actor<positionner::Controller<'static>>,
    piezostack: Actor<piezostack::Controller<'static>>,
    tiptilt_set_point: Initiator<Signals, FSM_RATE>,
    tiptilt: Actor<tiptilt::Controller<'static>, FSM_RATE>,
}

impl<const FSM_RATE: usize> M2System<FSM_RATE> {
    /// Creates M1 system with `n_step` time step
    pub fn new(n_step: usize) -> Self {
        Self {
            positionner_set_point: (Signals::new(42, n_step), "M2 Positionners 0pt").into(),
            positionner: (positionner::Controller::new(), "M2 Positionners").into(),
            piezostack: (piezostack::Controller::new(), "M2 PZT Actuators").into(),
            tiptilt_set_point: (
                Into::<Signals>::into((vec![0f64; 14], n_step)),
                "TipTilt_setpoint",
            )
                .into(),
            tiptilt: (tiptilt::Controller::new(), "M2 TipTilt Control").into(),
        }
    }
    /// Returns the tip-tilt controller
    pub fn tiptilt(&mut self) -> &mut Actor<tiptilt::Controller<'static>, FSM_RATE> {
        &mut self.tiptilt
    }
    /// Connects inputs and outputs of M2 system actors and return the vector of actors
    pub fn build(
        mut self,
        fem: &mut Actor<DiscreteModalSolver<ExponentialMatrix>>,
    ) -> Vec<Box<dyn Task>> {
        self.positionner_set_point
            .add_output()
            .build::<M2poscmd>()
            .into_input(&mut self.positionner);
        self.positionner
            .add_output()
            .build::<MCM2SmHexF>()
            .into_input(fem);
        self.piezostack
            .add_output()
            .build::<MCM2PZTF>()
            .into_input(fem);
        fem.add_output()
            .bootstrap()
            .build::<MCM2SmHexD>()
            .into_input(&mut self.positionner);
        fem.add_output()
            .bootstrap()
            .build::<MCM2PZTD>()
            .into_input(&mut self.piezostack);
        self.tiptilt_set_point
            .add_output()
            .build::<TTSP>()
            .into_input(&mut self.tiptilt);
        self.tiptilt
            .add_output()
            .bootstrap()
            .build::<PZTcmd>()
            .into_input(&mut self.piezostack);
        vec![
            Box::new(self.positionner_set_point),
            Box::new(self.positionner),
            Box::new(self.piezostack),
            Box::new(self.tiptilt_set_point),
            Box::new(self.tiptilt),
        ]
    }
}
