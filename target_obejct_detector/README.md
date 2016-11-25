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
