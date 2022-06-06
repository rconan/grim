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

## Running the model

```
sudo -E LD_LIBRARY_PATH=/usr/local/cuda/lib64 ./target/release/main
```
