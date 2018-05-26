# Camera LIDAR calibration
This document will look into methods of calibrating camera and LIDAR in order to collect high quality data with the [device](https://github.com/KnoxKang/KnoxKang-Company-work/blob/master/3D_Mapper.md) I made.

## Software used
[Autoware](https://github.com/CPFL/Autoware) : A toolbox for autonomous driving. It is capable of and not limited to 3D Localization, 3D Mapping, Path Planning, Path Following.

## How to calibrate camera & LIDAR
In order to calibrate camera and LIDAR, it is essential to understand how it works, and in this section, we will think camera and LIDAR already has their parameters known.
Camera & 3D LIDAR calibration has numerous ways to solve, but, all those methods have one thing in common. That is, whether if there is a 'target' or not, camera and LIDAR have to be able to 'see' the same distinct features. For example, if 'target', which can be a plane with distinct markings on it or some plane with distinct shape, is used, camera will recognize the target with visual disparity, and LIDAR will recognize the target with distance disparity from surrounding world. 
After target is recognized, extrinsic parameter between camera and LIDAR can be calculated in numerous different ways, and with extrinsic parameter in hand, camera & LIDAR calibration is done.

## How to calibrate camera & LIDAR in Autoware
In Autoware, this process is done by just running calibration toolkit under sensing tab. Once dialog box appears asking what kind of calibration do you want, select camera Velodyne calibration and proceed according to dialog box. After a new window appears, reorganize panel sizes, and press 'b' in LIDAR panel(upper/lower right panel) to adjust background color. Or else, there will be a black dot for LIDAR point cloud on black background. 
if you've selected a right topic, then you'll able to see camera feed on the upper left panel, and LIDAR point cloud on upper right panel. with this you can see if camera and LIDAR can see the same target or not. Press 'Grab' on top right row of buttons to capture image/point cloud set. When you capture it, camera image will appear on bottom left panel, and LIDAR point cloud of that instance will appear on the bottom right panel. 
Grab image/point cloud set from area where camera & LIDAR can both see on following points. upper left/right/center, lower left/right/center, center left/right/center, and on each point you have to grab at least two skewed and one straightly placed set, and repeat this process at least two times differing distance.
After all image/point cloud sets are captured, use your mouse to point and mark the most center of you target in captured LIDAR point cloud panel. The more center you click, the better the calibration will become. You can also cancel the marker placement by right clicking.
When you are done marking the center of your target on point cloud panel, click 'Calibrate' on top right row of buttons, and choose what you want in dialog window asking if you want to save captured image data and point cloud data. If you select that you want to save those data, you can later recalibrate by loading .yml file Autoware will produce through calibration.

## Things to do
- Modify Autoware's source code so that it can perform stereo camera & LIDAR calibration
-  Use the [device](https://github.com/KnoxKang/KnoxKang-Company-work/blob/master/3D_Mapper.md) to collect data
