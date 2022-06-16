use crate::{Error, Result};
use crseo::{
    calibrations, AtmosphereBuilder, Builder, Calibration, Gmt, GmtBuilder, ShackHartmannBuilder,
    Source, SourceBuilder,
};
use dos_actors::{
    clients::ceo::{OpticalModel, OpticalModelOptions, ShackHartmannOptions},
    Actor, Task,
};
use nalgebra as na;
use std::{iter::once, marker::PhantomData, time::Instant};

const U: usize = 1;

#[derive(Default)]
pub struct AGWS {
    gmt: GmtBuilder,
    maybe_atmosphere: Option<OpticalModelOptions>,
    maybe_dome_seeing: Option<OpticalModelOptions>,
    maybe_aberration: Option<OpticalModelOptions>,
}
impl AGWS {
    pub fn new() -> Self {
        Default::default()
    }
    pub fn atmosphere(mut self, builder: AtmosphereBuilder, time_step: f64) -> Self {
        self.maybe_atmosphere = Some(OpticalModelOptions::Atmosphere { builder, time_step });
        self
    }
    pub fn dome_seeing(mut self, cfd_case: String, upsampling_rate: usize) -> Self {
        self.maybe_dome_seeing = Some(OpticalModelOptions::DomeSeeing {
            cfd_case,
            upsampling_rate,
        });
        self
    }
    pub fn static_aberration(mut self, phase: Vec<f32>) -> Self {
        self.maybe_aberration = Some(OpticalModelOptions::StaticAberration(phase.into()));
        self
    }
    fn build_wfs<K: SH24orSH48, const R: usize>(
        &self,
        wfs: AgwsShackHartmann<K, R>,
    ) -> Result<Actor<OpticalModel, U, R>> {
        let options: Vec<OpticalModelOptions> = once(wfs.wfs.clone())
            .chain(self.maybe_atmosphere.clone())
            .chain(self.maybe_dome_seeing.clone())
            .chain(self.maybe_aberration.clone())
            .collect();
        let mut optical_model = OpticalModel::builder()
            .gmt(self.gmt.clone())
            .source(wfs.guide_star.clone())
            .options(options)
            .build()
            .map_err(|_| Error::OpticalModel)?;
        let pinv = wfs.poke(
            &mut optical_model.gmt,
            &mut optical_model.src,
            calibrations::ValidLensletCriteria::OtherSensor(
                &mut optical_model.sensor.as_mut().unwrap(),
            ),
        );
        optical_model.sensor_matrix_transform(pinv);
        Ok(optical_model.into())
    }
    pub fn build<const SH24_RATE: usize, const SH48_RATE: usize>(
        self,
        sh24: Option<AgwsShackHartmann<SH24, SH24_RATE>>,
        sh48: Option<AgwsShackHartmann<SH48, SH48_RATE>>,
    ) -> Result<Vec<Box<dyn Task>>> {
        let mut actors: Vec<Box<dyn Task>> = Vec::new();
        if let Some(wfs) = sh24 {
            actors.push(Box::new(self.build_wfs(wfs)?));
        }
        if let Some(wfs) = sh48 {
            actors.push(Box::new(self.build_wfs(wfs)?));
        }
        Ok(actors)
    }
}

type Poker = Vec<
    Option<
        Vec<(
            crseo::calibrations::Mirror,
            Vec<crseo::calibrations::Segment>,
        )>,
    >,
>;
pub struct AgwsShackHartmann<Kind, const R: usize> {
    guide_star: SourceBuilder,
    wfs: OpticalModelOptions,
    senses: Poker,
    n_sensor: usize,
    left_pinv: Option<na::DMatrix<f64>>,
    right_pinv: Option<na::DMatrix<f64>>,
    kind: PhantomData<Kind>,
}
impl<Kind: SH24orSH48, const R: usize> AgwsShackHartmann<Kind, R> {
    pub fn guide_star(mut self, guide_star: SourceBuilder) -> Self {
        self.guide_star = guide_star;
        self
    }
    pub fn flux_threshold(mut self, new_flux_threshold: f64) -> Self {
        if let OpticalModelOptions::ShackHartmann {
            ref mut flux_threshold,
            ..
        } = self.wfs
        {
            *flux_threshold = new_flux_threshold;
        }
        self
    }
    pub fn poker(mut self, senses: Poker) -> Self {
        self.senses = senses;
        self
    }
    fn poke(
        &self,
        gmt: &mut Gmt,
        src: &mut Source,
        valid_lenslet_criteria: calibrations::ValidLensletCriteria,
    ) -> na::DMatrix<f64> {
        let wfs_poker = <Kind as SH24orSH48>::wfs_poker().n_sensor(self.n_sensor);
        println!(" - calibration ...");
        let mut gmt2wfs = Calibration::new(gmt, src, wfs_poker);
        let now = Instant::now();
        gmt2wfs.calibrate(self.senses.clone(), valid_lenslet_criteria);
        println!(
            "GMT 2 WFS calibration [{}x{}] in {}s",
            gmt2wfs.n_data,
            gmt2wfs.n_mode,
            now.elapsed().as_secs()
        );

        let dof2wfs: Vec<f64> = gmt2wfs.poke.into();
        let dof2wfs = na::DMatrix::<f64>::from_column_slice(
            dof2wfs.len() / gmt2wfs.n_mode,
            gmt2wfs.n_mode,
            &dof2wfs,
        );
        let singular_values = dof2wfs.singular_values();
        let max_sv: f64 = singular_values[0];
        let min_sv: f64 = *singular_values.as_slice().iter().last().unwrap(); //.clone();
        let condition_number = max_sv / min_sv;
        println!("Poke matrix condition number: {condition_number:e}");
        let wfs2dof = dof2wfs
            //.clone()
            .pseudo_inverse(0f64)
            .expect("failed to compute poke matrix pseudo-inverse");
        match (self.left_pinv.as_ref(), self.right_pinv.as_ref()) {
            (Some(left), Some(right)) => left * wfs2dof * right,
            (Some(left), None) => left * wfs2dof,
            (None, Some(right)) => wfs2dof * right,
            (None, None) => wfs2dof,
        }
    }
}

pub trait AgwsShackHartmannBuilder<Type> {
    fn builder(n_sensor: usize) -> Self;
    fn poker_wfs(n_sensor: usize) -> ShackHartmannBuilder<crseo::Geometric>;
    //fn guide_star(self, guide_star: SourceBuilder) -> Self;
    //fn flux_threshold(self, flux_threshold: f64) -> Self;
}

pub enum SH24 {}
pub enum SH48 {}

pub trait SH24orSH48 {
    fn wfs_poker() -> ShackHartmannBuilder<crseo::Geometric>;
}
impl SH24orSH48 for SH24 {
    fn wfs_poker() -> ShackHartmannBuilder<crseo::Geometric> {
        *crseo::SH24::<crseo::Geometric>::new()
    }
}
impl SH24orSH48 for SH48 {
    fn wfs_poker() -> ShackHartmannBuilder<crseo::Geometric> {
        *crseo::SH48::<crseo::Geometric>::new()
    }
}

pub enum Geometric {}
pub enum Diffractive {}

pub trait GeometricOrDiffractive {}
impl GeometricOrDiffractive for Geometric {}
impl GeometricOrDiffractive for Diffractive {}

impl<const R: usize> AgwsShackHartmannBuilder<Geometric> for AgwsShackHartmann<SH24, R> {
    fn builder(n_sensor: usize) -> Self {
        Self {
            guide_star: Default::default(),
            wfs: OpticalModelOptions::ShackHartmann {
                options: ShackHartmannOptions::Geometric(
                    crseo::SH24::<crseo::Geometric>::new().n_sensor(n_sensor),
                ),
                flux_threshold: Default::default(),
            },
            n_sensor,
            senses: vec![],
            left_pinv: None,
            right_pinv: None,
            kind: PhantomData,
        }
    }
    fn poker_wfs(n_sensor: usize) -> ShackHartmannBuilder<crseo::Geometric> {
        crseo::SH24::<crseo::Geometric>::new().n_sensor(n_sensor)
    }
}
impl<const R: usize> AgwsShackHartmannBuilder<Diffractive> for AgwsShackHartmann<SH24, R> {
    fn builder(n_sensor: usize) -> Self {
        Self {
            guide_star: Default::default(),
            wfs: OpticalModelOptions::ShackHartmann {
                options: ShackHartmannOptions::Diffractive(
                    crseo::SH24::<crseo::Diffractive>::new().n_sensor(n_sensor),
                ),
                flux_threshold: Default::default(),
            },
            n_sensor,
            senses: vec![],
            left_pinv: None,
            right_pinv: None,
            kind: PhantomData,
        }
    }
    fn poker_wfs(n_sensor: usize) -> ShackHartmannBuilder<crseo::Geometric> {
        crseo::SH24::<crseo::Geometric>::new().n_sensor(n_sensor)
    }
}
impl<const R: usize> AgwsShackHartmannBuilder<Geometric> for AgwsShackHartmann<SH48, R> {
    fn builder(n_sensor: usize) -> Self {
        Self {
            guide_star: Default::default(),
            wfs: OpticalModelOptions::ShackHartmann {
                options: ShackHartmannOptions::Geometric(
                    *crseo::SH48::<crseo::Geometric>::new().n_sensor(n_sensor),
                ),
                flux_threshold: Default::default(),
            },
            n_sensor,
            senses: vec![],
            left_pinv: None,
            right_pinv: None,
            kind: PhantomData,
        }
    }
    fn poker_wfs(n_sensor: usize) -> ShackHartmannBuilder<crseo::Geometric> {
        *crseo::SH48::<crseo::Geometric>::new().n_sensor(n_sensor)
    }
}
impl<const R: usize> AgwsShackHartmannBuilder<Diffractive> for AgwsShackHartmann<SH48, R> {
    fn builder(n_sensor: usize) -> Self {
        Self {
            guide_star: Default::default(),
            wfs: OpticalModelOptions::ShackHartmann {
                options: ShackHartmannOptions::Diffractive(
                    *crseo::SH48::<crseo::Diffractive>::new().n_sensor(n_sensor),
                ),
                flux_threshold: Default::default(),
            },
            n_sensor,
            senses: vec![],
            left_pinv: None,
            right_pinv: None,
            kind: PhantomData,
        }
    }
    fn poker_wfs(n_sensor: usize) -> ShackHartmannBuilder<crseo::Geometric> {
        *crseo::SH48::<crseo::Geometric>::new().n_sensor(n_sensor)
    }
}
