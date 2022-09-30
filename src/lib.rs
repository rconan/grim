mod m1_system;
pub use m1_system::{M1System, Segment};

mod m2_system;
pub use m2_system::M2System;

pub mod agws;
#[doc(no_inline)]
pub use agws::AGWS;

pub mod control;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("failed to create segment actors")]
    Segment,
    #[error("failed to create modes 2 forces actor")]
    Mode2Force(#[from] m1_ctrl::M1Error),
    #[error("failed to build optical model")]
    OpticalModel,
}
pub type Result<T> = std::result::Result<T, Error>;
