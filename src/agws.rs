//! # GMT AGWS

use crate::{Error, Result};
use crseo::{
    calibrations::{self, Mirror, Segment},
    wavefrontsensor::Model,
    AtmosphereBuilder, Builder, Calibration, GmtBuilder, ShackHartmannBuilder, SourceBuilder,
};
use crseo_client::{OpticalModel, OpticalModelOptions, ShackHartmannOptions};
use dos_actors::Actor;
use nalgebra as na;
use std::{iter::once, marker::PhantomData, time::Instant};

const U: usize = 1;

pub fn pseudo_inverse(dof2wfs: na::DMatrix<f64>) -> na::DMatrix<f64> {
    let singular_values = dof2wfs.singular_values();
    let max_sv: f64 = singular_values[0];
    let min_sv: f64 = *singular_values.as_slice().iter().last().unwrap(); //.clone();
    let condition_number = max_sv / min_sv;
    println!("Poke matrix condition number: {condition_number:e}");
    let wfs2dof = dof2wfs
        //.clone()
        .pseudo_inverse(0f64)
        .expect("failed to compute poke matrix pseudo-inverse");
    wfs2dof
}

pub enum AgwsActors<const I: usize, const O24: usize, const O48: usize> {
    SH24(Actor<OpticalModel, I, O24>),
    SH48(Actor<OpticalModel, I, O48>),
    Both((Actor<OpticalModel, I, O24>, Actor<OpticalModel, I, O48>)),
    None,
}

impl<const I: usize, const O: usize> From<AgwsActors<I, O, O>> for Actor<OpticalModel, I, O> {
    fn from(wfs: AgwsActors<I, O, O>) -> Self {
        match wfs {
            AgwsActors::SH24(actor) => actor,
            AgwsActors::SH48(actor) => actor,
            AgwsActors::Both(_) => panic!("expected AGWS SH24 WFS, found both SH24 and SH48"),
            AgwsActors::None => panic!("expected either AGWS SH24 or SH48 WFS, found none"),
        }
    }
}

impl<const I: usize, const O24: usize, const O48: usize> From<AgwsActors<I, O24, O48>>
    for (Actor<OpticalModel, I, O24>, Actor<OpticalModel, I, O48>)
{
    fn from(wfs: AgwsActors<I, O24, O48>) -> Self {
        match wfs {
            AgwsActors::Both(actors) => actors,
            AgwsActors::SH24(_) => panic!("expected both AGWS SH24 and SH48 WFS, found SH24"),
            AgwsActors::SH48(_) => panic!("expected both AGWS SH24 and SH48 WFS, found SH48"),
            AgwsActors::None => panic!("expected both AGWS SH24 and SH48 WFS, found none"),
        }
    }
}

/// AGWS builder
#[derive(Default)]
pub struct AGWS {
    gmt: GmtBuilder,
    maybe_atmosphere: Option<OpticalModelOptions>,
    maybe_dome_seeing: Option<OpticalModelOptions>,
    maybe_aberration: Option<OpticalModelOptions>,
}
impl AGWS {
    /// Creates a bare AGWS builder with onlly the default [gmt optical model](crseo::GmtBuilder)
    pub fn new() -> Self {
        Default::default()
    }
    /// Sets the [gmt builder](crseo::GmtBuilder)
    pub fn gmt(mut self, gmt: GmtBuilder) -> Self {
        self.gmt = gmt;
        self
    }
    /// Sets the [atmosphere](crseo::AtmosphereBuilder)
    pub fn atmosphere(mut self, builder: AtmosphereBuilder, time_step: f64) -> Self {
        self.maybe_atmosphere = Some(OpticalModelOptions::Atmosphere { builder, time_step });
        self
    }
    /// Sets the [dome seeing](dos_actors::clients::ceo::OpticalModelOptions)
    pub fn dome_seeing(mut self, cfd_case: String, upsampling_rate: usize) -> Self {
        self.maybe_dome_seeing = Some(OpticalModelOptions::DomeSeeing {
            cfd_case,
            upsampling_rate,
        });
        self
    }
    fn maybe_tags(&self) -> String {
        let mut t = vec![];
        if self.maybe_atmosphere.is_some() {
            t.push(" . atmosphere");
        }
        if self.maybe_dome_seeing.is_some() {
            t.push(" . dome seeing");
        }
        if self.maybe_aberration.is_some() {
            t.push(" . static aberrations");
        }
        format!(r"{}", t.join(r"\n"))
    }
    /// Sets the [static aberrations](dos_actors::clients::ceo::OpticalModelOptions)
    pub fn static_aberration(mut self, phase: Vec<f32>) -> Self {
        self.maybe_aberration = Some(OpticalModelOptions::StaticAberration(phase.into()));
        self
    }
    pub fn poke_with<K, T, const R: usize>(
        &self,
        wfs: &AgwsShackHartmann<K, T, R>,
    ) -> Result<na::DMatrix<f64>>
    where
        K: SH24orSH48,
        T: GeometricOrDiffractive,
    {
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
        Ok(wfs.poke(&mut optical_model))
    }
    fn build_wfs<K, T, const R: usize>(
        &self,
        wfs: AgwsShackHartmann<K, T, R>,
    ) -> Result<Actor<OpticalModel, U, R>>
    where
        K: SH24orSH48,
        T: GeometricOrDiffractive,
    {
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
        if !wfs.is_poker_empty() {
            let wfs2dof = pseudo_inverse(wfs.poke(&mut optical_model));
            let pinv = match (wfs.maybe_left_pinv.as_ref(), wfs.maybe_right_pinv.as_ref()) {
                (Some(left), Some(right)) => left * wfs2dof * right,
                (Some(left), None) => left * wfs2dof,
                (None, Some(right)) => wfs2dof * right,
                (None, None) => wfs2dof,
            };
            optical_model.sensor_matrix_transform(pinv);
        }
        Ok((
            optical_model,
            format!(r"{}\n{}", wfs.tag(), self.maybe_tags()),
        )
            .into())
    }
    /// Builds and returns AGWS SH24 and SH48 wavefront sensors
    pub fn build<T24, T48, const SH24_RATE: usize, const SH48_RATE: usize>(
        self,
        sh24: Option<AgwsShackHartmann<SH24, T24, SH24_RATE>>,
        sh48: Option<AgwsShackHartmann<SH48, T48, SH48_RATE>>,
    ) -> AgwsActors<U, SH24_RATE, SH48_RATE>
    where
        T24: GeometricOrDiffractive,
        T48: GeometricOrDiffractive,
    {
        match (
            sh24.and_then(|wfs| self.build_wfs(wfs).ok()),
            sh48.and_then(|wfs| self.build_wfs(wfs).ok()),
        ) {
            (Some(actor_24), Some(actor_48)) => AgwsActors::Both((actor_24, actor_48)),
            (None, None) => AgwsActors::None,
            (None, Some(actor)) => AgwsActors::SH48(actor),
            (Some(actor), None) => AgwsActors::SH24(actor),
        }
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
/// AGWS Shack-Hartmann WFS builder
pub struct AgwsShackHartmann<Kind, Type, const R: usize> {
    guide_star: SourceBuilder,
    wfs: OpticalModelOptions,
    senses: Poker,
    n_sensor: usize,
    maybe_left_pinv: Option<na::DMatrix<f64>>,
    maybe_right_pinv: Option<na::DMatrix<f64>>,
    kind: PhantomData<Kind>,
    r#type: PhantomData<Type>,
}
impl<Kind, Type, const R: usize> AgwsShackHartmann<Kind, Type, R>
where
    Kind: SH24orSH48,
    Type: GeometricOrDiffractive,
{
    /// Create `n_sensor` bare AGWS Shack-Hartmann wavefront sensors of the [model type](GeometricOrDiffractive)
    ///
    /// A bare sensor uses an on-axis guide star with all the lenslets
    pub fn builder(n_sensor: usize) -> Self {
        Self {
            guide_star: Default::default(),
            wfs: OpticalModelOptions::ShackHartmann {
                options: <Type as GeometricOrDiffractive>::options::<Kind>(n_sensor),
                flux_threshold: Default::default(),
            },
            n_sensor,
            senses: vec![],
            maybe_left_pinv: None,
            maybe_right_pinv: None,
            kind: PhantomData,
            r#type: PhantomData,
        }
    }
    /// Sets the [guide stars(s)](SourceBuilder)
    pub fn guide_star(mut self, guide_star: SourceBuilder) -> Self {
        self.guide_star = guide_star;
        self
    }
    /// Sets the lenset intensity threshold to select *valid* lenslets
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
    /// Sets the [*observable* GMT modes](calibrations)
    pub fn poker(mut self, senses: Poker) -> Self {
        self.senses = senses;
        self
    }
    /// Sets the matrix that left-multiplies the pseudo-inverse of the poke matrix
    pub fn left_pinv(mut self, left: na::DMatrix<f64>) -> Self {
        self.maybe_left_pinv = Some(left);
        self
    }
    /// Sets the matrix that right-multiplies the pseudo-inverse of the poke matrix
    pub fn right_pinv(mut self, right: na::DMatrix<f64>) -> Self {
        self.maybe_right_pinv = Some(right);
        self
    }
    /*     pub fn poke(
        &self,
        gmt: &mut Gmt,
        src: &mut Source,
        valid_lenslet_criteria: calibrations::ValidLensletCriteria,
    ) -> na::DMatrix<f64> {
        let poker_wfs = <Kind as SH24orSH48>::poker_wfs().n_sensor(self.n_sensor);
        println!(" - calibration ...");
        let mut gmt2wfs = Calibration::new(gmt, src, poker_wfs);
        let now = Instant::now();
        gmt2wfs.calibrate(self.senses.clone(), valid_lenslet_criteria);
        println!(
            "GMT 2 WFS calibration [{}x{}] in {}s",
            gmt2wfs.n_data,
            gmt2wfs.n_mode,
            now.elapsed().as_secs()
        );
        let dof2wfs: Vec<f64> = gmt2wfs.poke.into();
        na::DMatrix::<f64>::from_column_slice(
            dof2wfs.len() / gmt2wfs.n_mode,
            gmt2wfs.n_mode,
            &dof2wfs,
        )
    } */
    pub fn is_poker_empty(&self) -> bool {
        self.senses.is_empty()
    }
    pub fn with_m2_tiptilt(mut self) -> Self {
        self.senses = vec![Some(vec![(Mirror::M2, vec![Segment::Rxyz(1e-6, Some(0..2))],)]); 7];
        self
    }
    fn poke(&self, optical_model: &mut OpticalModel) -> na::DMatrix<f64> {
        let poker_wfs = <Kind as SH24orSH48>::poker_wfs().n_sensor(self.n_sensor);
        println!(" - calibration ...");
        let mut gmt2wfs =
            Calibration::new(&mut optical_model.gmt, &mut optical_model.src, poker_wfs);
        let now = Instant::now();
        gmt2wfs.calibrate(
            self.senses.clone(),
            calibrations::ValidLensletCriteria::OtherSensor(
                &mut optical_model.sensor.as_mut().unwrap(),
            ),
        );
        println!(
            "GMT 2 WFS calibration [{}x{}] in {}s",
            gmt2wfs.n_data,
            gmt2wfs.n_mode,
            now.elapsed().as_secs()
        );
        let dof2wfs: Vec<f64> = gmt2wfs.poke.into();
        na::DMatrix::<f64>::from_column_slice(
            dof2wfs.len() / gmt2wfs.n_mode,
            gmt2wfs.n_mode,
            &dof2wfs,
        )
    }
    /// Returns the sensor kind
    pub fn tag(&self) -> String {
        if self.n_sensor == 1 {
            <Kind as SH24orSH48>::tag()
        } else {
            format!("{} x{}", <Kind as SH24orSH48>::tag(), self.n_sensor)
        }
    }
}

/// AGWS 24x24 Shack-Hartmann wavefront sensor type
pub enum SH24 {}
/// AGWS 48x48 Shack-Hartmann wavefront sensor type
pub enum SH48 {}

/// Interface to the kind of Shack-Hartmann wavefront sensor, [SH24] or [SH48]
pub trait SH24orSH48 {
    /// Returns the wavefront sensor used to compute the poke matrix
    ///
    /// This sensor is always of the [Geometric] type
    fn poker_wfs() -> ShackHartmannBuilder<crseo::Geometric>;
    /// Returns the sensor kind
    fn tag() -> String;
    /// Returns the wavefront sensor builder of type [T](Model)
    fn sensor<T: Model>() -> ShackHartmannBuilder<T>;
}
impl SH24orSH48 for SH24 {
    fn poker_wfs() -> ShackHartmannBuilder<crseo::Geometric> {
        *crseo::SH24::<crseo::Geometric>::new()
    }
    fn tag() -> String {
        "AGWS SH24".into()
    }
    fn sensor<T: Model>() -> ShackHartmannBuilder<T> {
        (*crseo::SH24::<T>::new()).clone()
    }
}
impl SH24orSH48 for SH48 {
    fn poker_wfs() -> ShackHartmannBuilder<crseo::Geometric> {
        *crseo::SH48::<crseo::Geometric>::new()
    }
    fn tag() -> String {
        "AGWS SH48".into()
    }
    fn sensor<T: Model>() -> ShackHartmannBuilder<T> {
        (*crseo::SH48::<T>::new()).clone()
    }
}

/// AGWS geometric Shack-Hartmann wavefront sensor model type
pub enum Geometric {}
/// AGWS diffractive Shack-Hartmann wavefront sensor model type
pub enum Diffractive {}

/// Interface to the type of Shack-Hartmann wavefront sensor, [Geometric] or [Diffractive]
pub trait GeometricOrDiffractive {
    /// Returns the wavefront sensor builder of kind [K](SH24orSH48) and type [T](Model)
    fn sensor<K, T>(n_sensor: usize) -> ShackHartmannBuilder<T>
    where
        K: SH24orSH48,
        T: Model,
    {
        <K as SH24orSH48>::sensor::<T>().n_sensor(n_sensor)
    }
    ///  Returns the sensor as a [ShackHartmannOptions] of an [OpticalModel] [options](OpticalModelOptions)
    fn options<K: SH24orSH48>(n_sensor: usize) -> ShackHartmannOptions;
}
impl GeometricOrDiffractive for Geometric {
    fn options<K: SH24orSH48>(n_sensor: usize) -> ShackHartmannOptions {
        ShackHartmannOptions::Geometric(Self::sensor::<K, crseo::Geometric>(n_sensor))
    }
}
impl GeometricOrDiffractive for Diffractive {
    fn options<K: SH24orSH48>(n_sensor: usize) -> ShackHartmannOptions {
        ShackHartmannOptions::Diffractive(Self::sensor::<K, crseo::Diffractive>(n_sensor))
    }
}
