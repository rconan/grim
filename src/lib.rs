mod m1_system;
pub use m1_system::{M1System, Segment};

pub mod agws;
#[doc(no_inline)]
pub use agws::AGWS;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("failed to create segment actors")]
    Segment,
    #[error("failed to create modes 2 forces actor")]
    Mode2Force(#[from] dos_actors::clients::m1::M1Error),
    #[error("failed to build optical model")]
    OpticalModel,
}
pub type Result<T> = std::result::Result<T, Error>;
