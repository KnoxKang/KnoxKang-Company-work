# Rig to map, reconstruct 3D environment.
This document will look into a rig capable of sensing 3D environment around the rig in form of point cloud via LRF, and apply texture to that point cloud from image data gathered by camera, with help of IMU measuring the rig's transformation and orientation.

## Devices used
- LRF (Laser Range Finder 'Velodyne VLP-16') : This equipment is here to provide point cloud data of 3D environmet of the rig.
- Camera (Point grey Black Fly * 2) : This equipment is for texture gathering purpose, and auxiliary range finding. 
- IMU (Inertial Measurement Unit 'SparkFun 9DoF Razor IMU M0') : This equipment is to let the rig be able to measure its movement (transformation, orientation). It also serves as an external triggering device for cameras and LRF.
- Tripod (Benro video camera tripod) : This equimpent holds all aformentioned equipments on a pan, tiltable mount.

## Softwares used
- ROS (Robot OS) : This software is the core software for this rig. In this rig, Kinetic Kame version is used.
  - ros Point grey camera driver : This is a driver needed to run two point grey cameras (If you want to change it to different camera(s), You'll need to find appropirate camera driver for that Camera).
  - ros Velodyne driver : This is a driver required to run Velodyne Lidar (If you want to change it to different LRF, You'll need to find right driver for that LRF).
  - ros Razor IMU driver : This is a driver needed to run Razor IMU from Sparkfun (If you want to change it to different IMU, You'll need to find corresponding driver for that IMU).
- Kalibr : This software is built by Autonomous Systems Lab and Skybotix AG. It is used in this rig to calibrate cameras, cameras with IMU, cameras with LRF (In case of camera-LRF calibration, You'll need to look into experimental branch).
- Arduino IDE : This developement environment is used in building this rig to customize IMU's firmware, So that IMU can function as a trigger device for cameras and LRF.

# How to build/use
- ## How to build
     - Hardware configuration can differ from person to person. Although, an example picture will be helpful.
     ![NEW_RIG](https://raw.githubusercontent.com/KnoxKang/KnoxKang-Company-work/master/Images/NEW_RIG.jpeg)
     - Software wise, You have a lot to take care of.
       - Download Ros from [Here](http://wiki.ros.org/kinetic/Installation) and install it as instructed.
       - Download Kalibr from [Here](https://github.com/ethz-asl/kalibr/wiki/installation#b-building-from-source) and build it as instructed. Reason for using source code building instead of CDE package is because you need to modify a bit of a code to make this rig work.
       - Download Point grey camera driver named Flycap2 SDK from [Here](https://www.ptgrey.com/support/downloads). You'll need to sign up on point grey site if you haven't.
       - Download ros Point grey camera driver from [Here](http://wiki.ros.org/pointgrey_camera_driver) and install it as instructed. Remember, you need to install version corresponding to your ros version (kinetic to knitic, lunar to lunar etc..)
       - Download ros Razor IMU driver from [Here](http://wiki.ros.org/razor_imu_9dof) and install it as instructed. Remember to use source install.
       - Download Arduino IDE from [Here](https://www.arduino.cc/en/Main/Software) and install it.
       - Reflash IMU's firmware using Arduino IDE using [This](https://github.com/KnoxKang/KnoxKang-Company-work/blob/master/Razor_AHRS.ino) source. This firmware allows IMU to trigger cameras, and Lidar.
       - In case you want to use raw format from point grey camera, modify  `elif data.encoding == "8UC1" or data.encoding == "mono8":` to `elif data.encoding == "8UC1" or data.encoding == "mono8" or data.encoding == "bayer_rggb8":` in `kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python/kalibr_common/ImageDatasetReader.py` at 135th line. This is to let `bayer_rgbg8` format converted to greyscale image used in Kalibr.
- ## How to use
     - Before starting, defalut usb buffer size(20mB) is not enough for point grey cameras. Follow [Configuring USBFS](https://www.ptgrey.com/tan/10685) instruction to clear that issue.
     - Starting off, run `roscore` in terminal. Reason for using seperate roscore rather than using roslaunch in other drivers and use their core is to let rviz to stay alive in case where you need to restart some of your drivers. i.e) just for caution.
     - Launch a new terminal and go to IMU_ws you've made in building phase. Run `source devel/setup.bash` and run `roslaunch razor_imu_9dof razor-pub.launch`. Now you'll have your imu publishing its message in `/imu` topic.
     - Launch a new terminal and run `roslaunch pointgrey_camera_driver stereo.launch left_camera_serial:=00000000 right_camera_serial:=00000000`. Replace 00000000 with your own camera serials. After this, you'll have your camera publishing its message in ros topic format.
     - Launch a nwe terminal and go to Velodyne_ws you've made in building phase. Run `source devel/setup.bash` and run `roslaunch velodyne_pointcloud VLP16_points.launch`. With this, you'll now have your Lidar publishing its mesage.
     - Launch a new terminal and run `rostopic list` and write down the topic you need to use. 
     - Use a terminal before, and run `rviz`. and now you can visualize all the topics you need to look in to like this. ![new_rig](https://raw.githubusercontent.com/KnoxKang/KnoxKang-Company-work/master/Images/New_Rig.png)
