# GMT Rust Integrated Model

The GRIM crate allows to run the GMT integrated model based on the [dos-actors](https://github.com/rconan/dos-actors) crate.

## Building the model

The complete model is build with
```
cargo build --release --features full
```
A simplified version with only the CFD wind loads (the first 10s) and the mount control system is build with
```
cargo build --release
```

## CFD remote drive install

To access the CFD wind loads remote drive, install the client for the Lustre FsX file system:
```
wget -O - https://fsx-lustre-client-repo-public-keys.s3.amazonaws.com/fsx-ubuntu-public-key.asc | sudo apt-key add -
```
```
sudo bash -c 'echo "deb https://fsx-lustre-client-repo.s3.amazonaws.com/ubuntu bionic main" > /etc/apt/sources.list.d/fsxlustreclientrepo.list && apt-get update'
```
```
sudo apt install -y linux-aws lustre-client-modules-aws && sudo reboot

```
wait a few minutes and log into the machine again.

## Model setup

The model is configured by setting some environment variables. Default values for all the variables can be set with
```
. setup.sh
```
The environment variables are:
 
 - FEM_REPO [fsx/20220308_1335_MT_mount_zen_30_m1HFN_FSM/]: the path to the GMT Finite Element state space model
 - GMT_MODES_PATH [/fsx/ceo]: the path to the GMT M1 and M2 CEO segment modes
 - M1CALIBRATION [/fsx/m1calibration/]: the path to M1 Finite Element sensitivity matrices
 - LOM [/fsx]: the path to the Linear Optical Model sensitivity matrices
 - DATA_REPO [/fsx/grim]: the path where the directory with the simulation results will be saved
  - SH48_N_STEP [5]: the number of 30s integration of the SH48 WFSs, the total simulated duration is: (10 + 30*SH48_N_STEP) seconds

An environment variable is set with
```
export <VAR_NAME>=<VAR_VALUE>
```

## Running the model

```
sudo -E LD_LIBRARY_PATH=/usr/local/cuda/lib64 ./target/release/main
```

## Model description

The model is sampled a 1kHz.

### FEM

The continuous FEM state space model is loaded from the data in the `$FEM_REPO` directory. The model is discretized for a sampling rate of 1kHz using the [matrix exponential](https://en.wikipedia.org/wiki/Discretization) method.

### Wind loads

The model applies CFD wind loads onto the FEM from the `zen30az000_OS7` CFD case.

### Mount control

The continuous control system of the mount drives has been discretized to match the simulation sampling rate of 1kHz.

### M1 hardpoints and actuators

The M1 force loop between M1 segment hardpoints and actuators is sampled at 100Hz.

### M2 positionners and piezo-stack actuators

Both control systems of M2 positionners and piezo-stack actuators are sampled at 1KHz.

### Tip-tilt loop

A feedback loop control system, sampled a 200Hz, between a 24x24 Shack-Hartmann WFS and the FSM tilt and tilt model is implemented. 

### Active Optics loop

A feedback loop control system, with a sampling rate of 30s, between a 48x48 Shack-Hartmann WFS and M1 and M2 rigid  body motions and M1 bending modes is implemented. 

