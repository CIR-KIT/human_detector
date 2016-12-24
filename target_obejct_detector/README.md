# Target object detector pkg

This is for detecting taget object(human) at Tsukuba Challenge 2016.

- [ ] コード重複多すぎ
- [ ] スケーリングパラメータのハードコートは良くない

## Requirements

- PCL 1.7+
- boost
- ROS(indigo)


## Usage
This package is using 3D pointcloud(pointcloud2) to recognize.

```
$ roslaunch target_object_detector target_object_detector.launch
```

- tf(/map, /base_link and sensor_frame)
- /amcl_pose (geometry_msgs/PoseWithCovarianceStamped)
- /hokuyo3d/hokuyo_cloud2

### Train
First, make `dataset/traian` directory in this pkg. Then move there.
```bash
roscd target_object_detector
mkdir -p dataset/train
cd dataset/train
```
Run the segmentation node in the directory.
```bash
rosrun target_object_detector segment_cluster_creator_node
```
Take a poingcloud by running the robot or play bag file include `pointclioud2` msg.
You will get a lot of pcd files in the directory.  
Next, classify the pcd files.
```bash
rosrun target_object_detector train_data_create_tool
```
After classified all pcd files, you will get `train.csv` in the directory.  
Third, making svm model.
```bash
roscd targe_object_detector/src/libsvm/tools/
python easy.py path/to/train.csv
```

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
- The target is ***NOT*** close to points where other targets are previously detected.

## Usage in GAZEBO

### 1. Start GAZEBO world with human models.

```bash
roslaunch cirkit_unit03_autorun autorun_gazebo_with_human.launch
```

### 2. Tune robot position with 2D Pose Estimate on Rviz.

### 3. Move Human models to an arbitary place on Rviz, if needed.

### 4. Run detector.

```bash
roslaunch target_object_detector target_object_detector.launch
```
