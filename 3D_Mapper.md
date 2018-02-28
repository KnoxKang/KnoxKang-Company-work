# Rig for 3D environment mapping and recontruction
This document will look into a rig capable of sensing 3D environment around the rig in form of point cloud via LRF, and apply texture to that point cloud from image data gathered by camera, with help of IMU measuring the rig's transformation and orientation.

## Devices used
- LRF (Laser Range Finder 'Velodyne VLP-16') : This equipment is here to provide point cloud data of 3D environmet of the rig.
- Camera (Point grey Black Fly * 2) : This equipment is for texture gathering purpose, and auxiliary range finding. 
- IMU (Inertial Measurement Unit 'SparkFun 9DoF Razor IMU M0') : This equipment is to let the rig be able to measure its movement (transformation, orientation). It also serves as an external triggering device for cameras and LRF.
- Tripod (Benro video camera tripod) : This equimpent holds all aformentioned equipments on a pan, tiltable mount.

## Softwares used
- [ROS (Robot OS)](http://www.ros.org/) : This software is the core software for this rig. In this rig, Kinetic Kame version is used.
  - [ros Point grey camera driver](http://wiki.ros.org/pointgrey_camera_driver) : This is a driver needed to run two point grey cameras (If you want to change it to different camera(s), You'll need to find appropirate camera driver for that Camera).
  - [ros Velodyne driver](http://wiki.ros.org/velodyne) : This is a driver required to run Velodyne Lidar (If you want to change it to different LRF, You'll need to find right driver for that LRF).
  - [ros Razor IMU driver](http://wiki.ros.org/razor_imu_9dof) : This is a driver needed to run Razor IMU from Sparkfun (If you want to change it to different IMU, You'll need to find corresponding driver for that IMU).
- [Kalibr](https://github.com/ethz-asl/kalibr) : This software is built by Autonomous Systems Lab and Skybotix AG. It is used in this rig to calibrate cameras, cameras with IMU, cameras with LRF (In case of camera-LRF calibration, You'll need to look into experimental branch).
- [Arduino IDE](https://www.arduino.cc/en/Main/Software) : This developement environment is used in building this rig to customize IMU's firmware, So that IMU can function as a trigger device for cameras and LRF.

# How to build/use
- ## How to build
     - Hardware configuration can differ from person to person. Although, an example picture will be helpful.
     ![NEW_RIG](https://github.com/KnoxKang/KnoxKang-Company-work/blob/master/Images/NEW_RIG.jpeg?raw=true) Notice that this picture doesn't has a IMU on it. IMU was added afterward on one of the camera arms.
     - Software wise, You have a lot to take care of.
       - Download Ros from [Here](http://wiki.ros.org/kinetic/Installation) and install it as instructed.
       - Download Kalibr from [Here](https://github.com/ethz-asl/kalibr/wiki/installation#b-building-from-source) and build it as instructed. Reason for using source code building instead of CDE package is because you need to modify a bit of a code to make this rig work.
       - Download Point grey camera driver named Flycap2 SDK from [Here](https://www.ptgrey.com/support/downloads). You'll need to sign up on point grey site if you haven't.
       - Download _ros Point grey camera driver_ from [Here](http://wiki.ros.org/pointgrey_camera_driver) and install it as instructed. Remember, you need to install version corresponding to your ros version (kinetic to knitic, lunar to lunar etc..)
       - Download _ros Razor IMU driver_ from [Here](http://wiki.ros.org/razor_imu_9dof) on section 4 and install it as instructed. Except, use imu_ws instead of catkin_ws. Remember to use source install.
       - Download Arduino IDE from [Here](https://www.arduino.cc/en/Main/Software) and install it.
       - Download Velodyne Lidar driver by running the following commands. 
       ```
       mkdir velodyne_ws
       cd velodyne_ws
       mkdir src
       git clone https://github.com/ros-drivers/velodyne.git
       cd ..
       catkin make
       ```
       - Reflash IMU's firmware using Arduino IDE using [This](https://github.com/KnoxKang/KnoxKang-Company-work/blob/master/Razor_AHRS.ino) source instead of original `Razor_AHRS.ino`. This custom firmware allows IMU to trigger cameras, and Lidar.
       - In case you want to use raw format from point grey camera, modify  `elif data.encoding == "8UC1" or data.encoding == "mono8":` to `elif data.encoding == "8UC1" or data.encoding == "mono8" or data.encoding == "bayer_rggb8":` in `kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python/kalibr_common/ImageDatasetReader.py` at 135th line. This is to let `bayer_rgbg8` format converted to greyscale image used in Kalibr.
- ## How to use
     - Before starting, default usb buffer size(20mB) is not enough for point grey cameras. Follow [Configuring USBFS](https://www.ptgrey.com/tan/10685) instruction to clear that issue.
     - Starting off, run `roscore` in terminal. Reason for using seperate roscore rather than using roslaunch in other drivers and use their core is to let rviz to stay alive in case where you need to restart some of your drivers. i.e. just for caution.
     - Launch a new terminal and go to imu_ws you've made in building phase. Run `source devel/setup.bash` and run `roslaunch razor_imu_9dof razor-pub.launch`. Now you'll have your imu publishing its message in `/imu` topic.
     - Launch a new terminal and run `roslaunch pointgrey_camera_driver stereo.launch left_camera_serial:=00000000 right_camera_serial:=00000000`. Replace 00000000 with your own camera serials. After this, you'll have your camera publishing its message in ros topic format.
     - Launch a new terminal and go to velodyne_ws you've made in building phase. Run `source devel/setup.bash` and run `roslaunch velodyne_pointcloud VLP16_points.launch`. With this, you'll now have your Lidar publishing its mesage.
     - Launch a new terminal and run `rostopic list` and write down topics you need to use. 
     - Use a terminal before, and run `rviz`. and now you can visualize all the topics you need to look into like this. ![new_rig](https://raw.githubusercontent.com/KnoxKang/KnoxKang-Company-work/master/Images/New_Rig.png)
     - Before you proceed, if you are not familiar with camera calibration, refer to [this](https://github.com/KnoxKang/KnoxKang-Company-work/blob/master/3D_Mapper.md#tips-and-tricks-for-calibration) and launch a new terminal and run `rosbag record /camera/left/image_SOMETHING /camera/left/image_SOMETHING -O static.bag` to record data for static calibration.
     - Run `rosbag record /camera/left/image_SOMETHING /camera/left/image_SOMETHING /imu -O dynamic.bag` to record data for dynamic calibration.
     - Run `source (Kalibr workspace you've made)/devel/setup.bash` and run `kalibr_calibrate_cameras --bag static.bag --target april_6x6_50x50cm.yaml --models pinhole-equi pinhole-equi --topics /camera/left/image_SOMETHING /camera/left/image_SOMETHING` to calibrate your cameras. This process will take a long time. 
     - Check the results plotted on screen. Remember, good calibration will come up with a reprojection error result tightly packed in a square less than 1 pixel per each side, and final reprojection error, which is shown in results-cam-static.txt in the directory you've ran a preceding command, has to be less than 0.2 pixel.
     - Run `source (Kalibr workspace you've made)/devel/setup.bash` if you are using new terminal, and run `kalibr_calibrate_imu_camera --target april_6x6_50x50cm.yaml --cam camchain-static.yaml --imu (IMU yaml file location) --bag dynamic.bag` to calibrate your cameras with IMU. This process will take lesser time than static calibration.
     
## Tips and tricks for calibration
As for static calibration, You have a chance of your final reprojection error never goes under 0.2 pixel, or overall error results are never packed in 1 pixel square, or both. In order to adress this, you'll need to understand why do you need to calibrate your camers, and how does it work. All cameras will receive disorted image of the world if it has a lens. Thus, if you need to use it in computer vision, you need to undistort the image your camera is receiving. Static calibration does just that by looking at the corners of your april target, which is known to be on a straight line, and try to match those corner points in straight line by distorting the image in oposite way the lens distorts the image. After that procedure is done, calibration algorythm will reproject the corner spot to a infinitly far awy sky sphere, comming up with reprojection error result. <br><br> In conclusion, to get a best calibraion (which means you can omit one or two. like light condition), you'll need to 
- Shorten your camera's shutter time to minimum as much as environmental lighiting will allow. This is to let cameras grab the sharpest image possible.
- Be in a very good light condition. This is to further shorten the shutter time.
- Go very close to the cameras.
- Go very far untill your cameras almost can't read your target.
- Go from left to right most angle from your cameras.
- Scew (tilting your target in axis that are not parallel with imaginary line going through the center of camera sensor and the center of camera lens) your target as much as possible (Actually, you only need to scew it upward and span all over the place).

This is the same procedure you might have done it, but, there's more catches.
- You have to fill imaginary space where your camera can see (calling it 'view space' from now on), comprised with left most and right most side of you cameara's field of view and distance between you camera, as thoroughly as possible. This could be done by moving slowly from back to from all over the view space. Followed by doing the same with scewed target.

Problem for this method is that the static.bag file becomes too big, costing a big chunk of time on static calibration. In order to avoid this issue
- You have to lower your camera's framerate (to about 10fps). 
- If you are up to, lower you camera's resolution (Be aware that if you do this you'll have to stay at that resolution)

## Things to do
- Make an algorythm that can acknowledge planes and use camera image data to apply texture on it.
- Make tf tree to recognize rig's movement. Thus, allow mapping.
- Fix Kalibr's broken Camera-LRF calibration code.
- Make all this procedure automated, so that using these won't be a problem for others.
