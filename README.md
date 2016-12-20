# human_detector [![Build Status](https://travis-ci.org/CIR-KIT/human_detector.svg?branch)](https://travis-ci.org/CIR-KIT/human_detector) [![Slack](https://img.shields.io/badge/Slack-CIR--KIT-blue.svg)](http://cir-kit.slack.com/messages/human_detector)

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

## Usage
### Human detection
##### Actual detection :
```
roslaunch target_object_detector target_object_detector.launch
```
##### Fake detaction： 
You can utilied `fake_target_detector` to assume a target is virtually detected, if you just check navigation behaivior ***without actual human detection.***  

A virtually detected target human point can be set by clicking a point in a map with `Publish point` in `Rviz`.

The following command shows a coordinate of clicked point.

```bash
rostopic echo /clicked_point
```

Save the coordinate `x, y` to`targetlist/targetlist.csv` ([a sample file](https://github.com/CIR-KIT/human_detector/blob/mm/add/document/fake_target_detector/targetfiles/targetlist.csv)).

To place an virtual target, run the following command.

```bash
rosrun fake_target_detector fake_target_detector
```

Bounding boxes will be showin at positions specified in `targetlist.csv` and the virtually detected positions are also to be published.

##### Common specification : 
Satisfying all of the following conditions invoke approching to a target.

- A currently reached waypoint is placed in detecting area.
- A target human is within 5 [m] from the robot.
- The target is ***NOT** close to points where other targets are previously detected.

## Usage in GAZEBO

### 1. Start GAZEBO world with human models.

```bash
roslaunch third_robot_2dnav_gazebo autorun_with_human.launch
```

### 2. Tune robot position with 2D Pose Estimate on Rviz.

### 3. Move Human models to an arbitary place on Rviz, if needed.

### 4. Run detector.

```bash
roslaunch target_object_detector target_object_detector.launch
```
