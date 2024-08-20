# omni_vision

**A ROS development package for tools and features related to omnidirectional cameras**

## Installation

Currently this package has been tested with ROS noetic running on Unbuntu 20.04. Please make sure to have a ROS [installation](https://wiki.ros.org/ROS/Installation) ready to go. To install, follow these standard steps.

```bash
cd ~/catkin_ws/src
git clone https://github.com/akorycki/omni_vision.git
cd ..
catkin_make
source devel/setup.bash
  bash
```

## Nodes

  - [**omni_rect**](#omni_rect)

## omni_rect
This node undistorts ROS image messages using omnidirectional camera intrinsic parameters and a standard radial-tangential distortion model. The node takes a calibration YAML file provided by the [Kalibr](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration) calibration tool. The yaml file follows the following format.

```text
cam0:
  cam_overlaps: []
  camera_model: omni
  distortion_coeffs: [k1, k2, r1, r2]
  distortion_model: radtan
  intrinsics: [xi, fx, fy, cx, cy]
  resolution: [width, height]
  rostopic: /my_camera/image_raw
```

Additionally, the omni_rect node takes in the following ROS parameters.

```text
new_width<int>:  width of undistorted image 
new_height<int>: height of undistorted image 
rectification_type<string>: type of rectification [perspective, cylindrical, stereographic, longlat]
```

For more information on the rectification types, refer to https://docs.opencv.org/4.x/dd/d12/tutorial_omnidir_calib_main.html. An example launch file is proviided for a Kodak PIXPRO ORBIT360 4K camera. To run this node, use the following command.

```bash
roslaunch omni_vision kodak_orbit360.launch
  bash
```
