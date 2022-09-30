# run a terminal with: . setup.sh
sudo mkdir -p /fsx && sudo mount -t lustre -o noatime,flock fs-0e6759f50ff7a310c.fsx.us-west-2.amazonaws.com@tcp:/x346hbmv /fsx
export M1CALIBRATION=/fsx/m1calibration/
export GMT_MODES_PATH=/fsx/ceo
export FEM_REPO=/fsx/Rodrigo/20220610_1023_MT_mount_zen_30_m1HFN_FSM_
export SH48_N_STEP=5
export LOM=/fsx
export DATA_REPO=/fsx/grim