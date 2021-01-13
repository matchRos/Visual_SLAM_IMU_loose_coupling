# Visual_SLAM_IMU_loose_coupling
This work deals with localization of mobile robot. ORB_SLAM and IMU are fused by using extended Kalman filter. 
This work is based on [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3), [robot_pose_ekf](https://github.com/ros-planning/robot_pose_ekf), 
[imu_calibration](https://github.com/gaowenliang/imu_utils) and [unified_calibration](https://github.com/ethz-asl/kalibr).
## Table of Contents
- [Environment](#Environment)
- [Hardware](#Hardware)
- [Installation](#Installation)
- [Calibration](#Calibration)
   - [IMU calibration](#IMU-calibration)
   - [Camera intrinsics calibration](#Camera-intrinsics-calibration)
   - [Unified calibration](#Unified-calibration)
- [Fusion](#Fusion)

## Environment
Ubuntu 18.04

ROS melodic
## Hardware
Realsense D435i
## Installation
install librealsense
```sh
./install_script_chen
```
install Eigen3: (http://eigen.tuxfamily.org/index.php?title=Main_Page)

install Ceres Solver: (http://ceres-solver.org/installation.html)

create catkin workspace:
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws 
catkin_make
source ~/catkin_ws/devel/setup.bash
```
install dependencies for kalibr
```sh
sudo apt-get install python-setuptools python-rosinstall ipython libeigen3-dev libboost-all-dev doxygen libopencv-dev ros-melodic-opencv ros-melodic-image-transport-plugins ros-melodic-cmake-modules python-software-properties software-properties-common libpoco-dev python-matplotlib python-scipy python-git python-pip ipython libtbb-dev libblas-dev liblapack-dev python-catkin-tools libv4l-dev
```
install repository:
```sh
cd ~/catkin_ws/src
git clone https://github.com/hustchenli/Visual_SLAM_IMU_loose_coupling.git
cd ..
catkin_make
```
install ORB_SLAM3:
```sh
cd ~/catkin_ws/src/ORB_SLAM3
chmod +x build.sh
./build.sh
gedit ~/.bashrc
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/PATH/ORB_SLAM3/Examples/ROS
chmod +x build_ros.sh
./build_ros.sh
```
install robot_pose_ekf
```sh
rosdep install robot_pose_ekf
roscd robot_pose_ekf
rosmake
```
## Calibration
### IMU calibration
Find the luanuch file of camera `rs_camera.launch`, and change the value of `unite_imu_method` to `linear_interpolation`:
```bash
<arg name="unite_imu_method"      default="linear_interpolation"/>
```
Plug camera to computer and run:
```sh
roscore 
roslaunch realsense2_camera rs_camera.launch
rostopic list 
```
check if there is topic `/camera/imu`

then keep the camera still, and record the data form imu as long as possible:
```sh
roscore 
roslaunch realsense2_camera rs_camera.launch
rosbag record -O imu_calibration /camera/imu 
```
change the value of `max_time_min` in `~/imu_utils/launch/d435i_imu_calibration.launch` to the length of the recording (unit:min), then run:
```sh
roslaunch imu_utils d435i_imu_calibration.launch
rosbag play -r 50 imu_calibration.bag
```
the result of IMU calibration is published in `~/imu_utils/data/d435i_imu_calibration_imu_param.yaml`.
### Camera intrinsics calibration
download and print calibration target: (https://github.com/ethz-asl/kalibr/wiki/downloads) and create yaml file, if the target is printed on A4 paper, yaml file should be:
```bash
target_type: 'aprilgrid' #gridtype
tagCols: 6               #number of apriltags
tagRows: 6               #number of apriltags
tagSize: 0.021           #size of apriltag, edge to edge [m]
tagSpacing: 0.3          #ratio of space between tags to tagSize
```
point the camera at the calibration target, and try to keep the calibration target in the camera’s field of view. Translate or rotate the camera and record image data for one to two minutes:
```sh
roscore 
roslaunch realsense2_camera rs_camera.launch
rosrun topic_tools throttle messages /camera/color/image_raw 4.0 /color
rosbag record -O camd435i /color
```
run the calibration programm:
```sh
kalibr_calibrate_cameras --target /PATH/April.yaml --bag /PATH/camd435i.bag --bag-from-to 20 110 --models pinhole-radtan --topics /color --show-extraction
```
then the calibration result would be published in yaml, txt and pdf file.
### Unified calibration
move the camera for one to two minutes, keep calibration target in camera’s field of view, record data from camera and IMU:
```sh
roscore
rosrun topic_tools throttle messages /camera/color/image_raw 20.0 /color
rosrun topic_tools throttle messages /camera/imu 200.0 /imu
rosbag record -O dynamic /color /imu
```
after recording, run:
```sh
kalibr_calibrate_imu_camera --target /PATH/April.yaml --cam /PATH/camera.yaml --imu /PATH/imu.yaml --bag /PATH/dynamic.bag --show-extraction
```
in file `camchain-imucam-dynamic.yaml` you will find transformation matrix between camera and IMU.
## Fusion
change the value of parameter `T_cam_imu` in `camimu_trans.py` according to the result in `camchain-imucam-dynamic.yaml`, change the camera intrinsics in `~/ORB_SLAM3/D435i.yaml` according to calibration results

turn on the camera:
```sh
roscore
roslaunch realsense2_camera rs_camera.launch
```
run:
```sh
python camimu_trans.py
```
turn on extended Kalman filter:
```sh
roslaunch robot_pose_ekf.launch
```
turn on ORB_SLAM3:
```sh
rosrun ORB_SLAM3 RGBD /PATH/ORB_SLAM3/Vocabulary/ORBvoc.txt /PATH/ORB_SLAM3/D435i.yaml
```
estimated pose from ORB_SLAM3 without fusion is:
```sh
rostopic echo /camera_pose
```
estimated pose from fusion of ORB_SLAM3 and IMU is:
```sh
rostopic echo /robot_pose_ekf/odom_combined
```

