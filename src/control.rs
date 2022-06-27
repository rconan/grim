pub trait Control {
    fn step(&mut self, value: &[f64]) -> Option<Vec<f64>>;
}

#[derive(Default)]
pub struct Average {
    n_sample: usize,
    average: Vec<f64>,
    counter: usize,
}
impl Average {
    pub fn new(n_sample: usize, n_data: usize) -> Self {
        Self {
            n_sample,
            average: vec![0f64; n_data],
            counter: 0,
        }
    }
}
impl Iterator for Average {
    type Item = Vec<f64>;
    fn next(&mut self) -> Option<Self::Item> {
        self.counter += 1;
        if self.counter == self.n_sample {
            let mean: Vec<_> = self
                .average
                .iter()
                .map(|x| x / self.n_sample as f64)
                .collect();
            self.average = vec![0f64; self.average.len()];
            self.counter = 0;
            Some(mean)
        } else {
            None
        }
    }
}
impl Control for Average {
    fn step(&mut self, value: &[f64]) -> Option<Vec<f64>> {
        self.average.iter_mut().zip(value).for_each(|(a, v)| {
            *a += *v;
        });
        self.next()
    }
}

#[derive(Default)]
pub struct Integrate {
    gain: f64,
    mem: Vec<f64>,
}
impl Integrate {
    pub fn new(gain: f64, n_data: usize) -> Self {
        Self {
            gain,
            mem: vec![0f64; n_data],
        }
    }
    pub fn last(&self) -> Option<Vec<f64>> {
        Some(self.mem.clone())
    }
}
impl Control for Integrate {
    fn step(&mut self, value: &[f64]) -> Option<Vec<f64>> {
        let gain = self.gain;
        self.mem.iter_mut().zip(value).for_each(|(a, v)| {
            *a += *v * gain;
        });
        self.last()
    }
}
#[derive(Default)]
pub struct Proportional {
    gain: f64,
    mem: Vec<f64>,
}
impl Proportional {
    pub fn new(gain: f64, n_data: usize) -> Self {
        Self {
            gain,
            mem: vec![0f64; n_data],
        }
    }
    pub fn last(&self) -> Option<Vec<f64>> {
        Some(self.mem.clone())
    }
}
impl Control for Proportional {
    fn step(&mut self, value: &[f64]) -> Option<Vec<f64>> {
        let gain = self.gain;
        self.mem.iter_mut().zip(value).for_each(|(a, v)| {
            *a = *v * gain;
        });
        self.last()
    }
}

#[derive(Default)]
pub struct Delay {
    mem: Vec<Vec<f64>>,
}
impl Delay {
    pub fn new(n_sample: usize, n_data: usize) -> Self {
        Self {
            mem: vec![vec![0f64; n_data]; n_sample + 1],
        }
    }
}
impl Control for Delay {
    fn step(&mut self, value: &[f64]) -> Option<Vec<f64>> {
        self.mem[0].copy_from_slice(value);
        self.mem.rotate_right(1);
        Some(self.mem[0].clone())
    }
}

#[derive(Default)]
pub struct StateSpace2x2 {
    a: [f64; 4],
    b: [f64; 2],
    c: [f64; 2],
    d: f64,
    x: (Vec<f64>, Vec<f64>),
}
impl StateSpace2x2 {
    pub fn new(a: [f64; 4], b: [f64; 2], c: [f64; 2], d: f64, n_data: usize) -> Self {
        Self {
            a,
            b,
            c,
            d,
            x: (vec![0f64; n_data], vec![0f64; n_data]),
        }
    }
}
impl Control for StateSpace2x2 {
    fn step(&mut self, u: &[f64]) -> Option<Vec<f64>> {
        let x_iter = (&self.x.0).into_iter().zip((&self.x.1).into_iter());
        let y: Vec<_> = u
            .iter()
            .zip(x_iter.clone())
            .map(|(u, x)| self.c[0] * x.0 + self.c[1] * x.1 + self.d * u)
            .collect();
        self.x = u
            .iter()
            .zip(x_iter)
            .map(|(u, x)| {
                (
                    self.a[0] * x.0 + self.a[1] * x.1 + self.b[0] * u,
                    self.a[2] * x.0 + self.a[3] * x.1 + self.b[1] * u,
                )
            })
            .unzip();
        Some(y)
    }
}

pub struct M2Dynamics {
    /// pole frequency [Hz]
    pub pole: f64,
    /// damping
    pub damping: f64,
    /// sensor integration time [rate]
    pub integration: usize,
    /// latency [rate]
    pub latency: usize,
    /// gain
    pub gain: f64,
    sensor: Average,
    integrator: Integrate,
    delay: Delay,
    dynamics: StateSpace2x2,
}
impl M2Dynamics {
    pub fn new(
        n: usize,
        pole: f64,
        damping: f64,
        integration: usize,
        latency: usize,
        gain: f64,
        ss: ([f64; 4], [f64; 2], [f64; 2], f64),
    ) -> Self {
        Self {
            pole,
            damping,
            integration,
            latency,
            gain,
            sensor: Average::new(integration, n),
            integrator: Integrate::new(gain, n),
            delay: Delay::new(latency, n),
            dynamics: StateSpace2x2::new(ss.0, ss.1, ss.2, ss.3, n),
        }
    }
    pub fn fsm(n: usize) -> Self {
        Self::new(
            n,
            25.,
            0.6,
            5,
            6,
            0.3,
            (
                [1.80628279, -0.82870523, 1., 0.],
                [1., 0.],
                [0.02133653, 0.00096021],
                0.00560561,
            ),
        )
    }
    pub fn asm(n: usize) -> Self {
        Self::new(
            n,
            800.,
            0.75,
            2,
            1,
            0.4,
            (
                [-0.95910647, -0.31990701, 1., 0.],
                [1., 0.],
                [0.5930526, 0.38748527],
                0.56975337,
            ),
        )
    }
    pub fn piston_optical_sensor(n: usize) -> Self {
        Self::new(
            n,
            800.,
            0.75,
            30_000,
            6,
            0.5,
            (
                [-0.95910647, -0.31990701, 1., 0.],
                [1., 0.],
                [0.5930526, 0.38748527],
                0.56975337,
            ),
        )
    }
    pub fn piston_edge_sensor(n: usize) -> Self {
        Self::new(
            n,
            800.,
            0.75,
            2,
            0,
            0.8,
            (
                [-0.95910647, -0.31990701, 1., 0.],
                [1., 0.],
                [0.5930526, 0.38748527],
                0.56975337,
            ),
        )
    }
    pub fn step(&mut self, u: &[f64]) -> Option<Vec<f64>> {
        /*         match self.sensor.step(input) {
            Some(ref value) => self.integrator.step(value),
            None => self.integrator.last(),
        }
        .map(|x| self.delay.step(&x).unwrap())
        .map(|x| self.dynamics.step(&x).unwrap()) */
        self.delay.step(u).and_then(|u| self.dynamics.step(&u))
    }
    pub fn es_step(&mut self, input: &[f64]) -> Option<Vec<f64>> {
        match self.sensor.step(input) {
            Some(ref value) => self.integrator.step(value),
            None => self.integrator.last(),
        }
        .map(|x| self.delay.step(&x).unwrap())
        .map(|x| self.dynamics.step(&x).unwrap())
    }
}

pub struct EdgeSensorPistonControl {
    /// pole frequency [Hz]
    pub pole: f64,
    /// damping
    pub damping: f64,
    /// sensor integration time [rate]
    pub integration: usize,
    /// latency [rate]
    pub latency: usize,
    /// gain
    pub gain: f64,
    sensor: Average,
    proportional: Proportional,
    delay: Delay,
    dynamics: StateSpace2x2,
}
impl EdgeSensorPistonControl {
    pub fn new(
        n: usize,
        pole: f64,
        damping: f64,
        integration: usize,
        latency: usize,
        gain: f64,
        ss: ([f64; 4], [f64; 2], [f64; 2], f64),
    ) -> Self {
        Self {
            pole,
            damping,
            integration,
            latency,
            gain,
            sensor: Average::new(integration, n),
            proportional: Proportional::new(gain, n),
            delay: Delay::new(latency, n),
            dynamics: StateSpace2x2::new(ss.0, ss.1, ss.2, ss.3, n),
        }
    }
    pub fn fsm(n: usize) -> Self {
        Self::new(
            n,
            25.,
            0.6,
            5,
            6,
            0.3,
            (
                [1.80628279, -0.82870523, 1., 0.],
                [1., 0.],
                [0.02133653, 0.00096021],
                0.00560561,
            ),
        )
    }
    pub fn asm(n: usize) -> Self {
        Self::new(
            n,
            800.,
            0.75,
            2,
            1,
            0.4,
            (
                [-0.95910647, -0.31990701, 1., 0.],
                [1., 0.],
                [0.5930526, 0.38748527],
                0.56975337,
            ),
        )
    }
    pub fn piston_optical_sensor(n: usize) -> Self {
        Self::new(
            n,
            800.,
            0.75,
            30_000,
            6,
            0.5,
            (
                [-0.95910647, -0.31990701, 1., 0.],
                [1., 0.],
                [0.5930526, 0.38748527],
                0.56975337,
            ),
        )
    }
    pub fn piston_edge_sensor(n: usize) -> Self {
        Self::new(
            n,
            800.,
            0.75,
            2,
            0,
            0.8,
            (
                [-0.95910647, -0.31990701, 1., 0.],
                [1., 0.],
                [0.5930526, 0.38748527],
                0.56975337,
            ),
        )
    }
    pub fn step(&mut self, input: &[f64]) -> Option<Vec<f64>> {
        match self.sensor.step(input) {
            Some(ref value) => self.proportional.step(value),
            None => self.proportional.last(),
        }
        .map(|x| self.delay.step(&x).unwrap())
        .map(|x| self.dynamics.step(&x).unwrap())
    }
}
