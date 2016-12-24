# human_detector [![Build Status](https://travis-ci.org/CIR-KIT/human_detector.svg?branch=indigo-devel)](https://travis-ci.org/CIR-KIT/human_detector) [![Slack](https://img.shields.io/badge/Slack-CIR--KIT-blue.svg)](http://cir-kit.slack.com/messages/human_detector)

## Summary
Human detector packages for Tshukuba Challenge indluding detector, sensor tuning and human model.

- target_obejct_detector : Core program for human detection.
- fake_target_detector : Fake detector for debugging.
- point_cloud_reducer : Reduce the number of points in the cloud to discrease computation time if needed.
- human_model_gazebo : Human model (URDF) to be detected in GAZEBO.

## Installation
##### 1. Create **catkinized**  workspace.
##### 2. Clone this repository.
```bash
$ cd <catkin_ws>/src
$ git clone https://github.com/CIR-KIT/human_detector.git
```
##### 3. Download required packages by wstool.
```bash
$ cd <catkin_ws>
$ wstool init src
$ wstool merge -t src src/human_detector/human_detector.rosinstall
$ wstool update -t src
```
##### 4. Download depended packages by rosdep.
```bash
$ cd <catkin_ws>
$ rosdep install -i -r -y --from-paths src --ignore-src
```
##### 5. Build packages, and set the path for the packages.
```bash
$ cd <catkin_ws>
$ catkin_make
$ source devel/setup.bash
```
