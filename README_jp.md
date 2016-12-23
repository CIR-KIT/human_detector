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
### 探索対象をみつける  
##### 実機の場合：  
```
roslaunch target_object_detector target_object_detector.launch
```
##### 実際に実験出来ない時：  
`fake_target_detector`を使って探索対象が見つかったことにできます。  
まず、どこに探索対象がいることにするかを決めます。地図を`rviz`で表示しながら探索対象が居る場所に`Publish point`を使ってクリックします。
```bash
rostopic echo /clicked_point
```
をすればクリックした座標がわかります。その`x, y`座標を`targetlist/targetlist.csv`におきます。
```bash
rosrun fake_target_detector fake_target_detector
```
とすれば`targetlist.csv`に書かれた座標にBoundingBoxが表示されているはずです。そうすればちゃんとpublishもされています。  
到達したwaypointが探索エリアでロボットから5[m]以内に探索対象がいればアプローチするはずです。  
また一度アプローチした探索対象から近い場合には無視します。  

## Usage in Gazebo

### 1. 人物付きでGazeboを起動する.

```bash
roslaunch third_robot_2dnav_gazebo autorun_with_human.launch
```

人を追加したかったら、上記launchを参考に追加して下さい。

### 2. Rviz 上で2D Pose Estimate で初期位置を修正する。

### 3. Rviz 上で人の位置を好きに移動させる。
色々な場所で認識させたい場合は、Gazeboで一時停止して移動→探索　を繰り返して使いまわすといいかもしれません。

### 4. detectorを起動する。

```bash
roslaunch target_object_detector target_object_detector.launch
```
