# human_detector
Human detector package for Tshukuba Challenge

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

## How to use in Gazebo?

### 1. 人物付きでGazeboを起動する.

```bash
roslaunch third_robot_2dnav_gazebo autorun_with_human.launch
```

人を追加したかったら，上記launchを参考に追加して下さい．

### 2. Rviz 上で2D Pose Estimate で初期位置を修正する．

下記コマンドで可視化した時の最初のwaypointとほぼ同じ場所．

```bash
rosrun ros_waypoint_generator ros_waypoint_generator --load 2016-11-01-04-01-48.csv
```

### 3. Rviz 上で人の位置を好きに移動させる．
色々な場所で認識させたい場合は，Gazeboで一時停止して移動→探索　を繰り返して使いまわすといいかもしれません．

### 4. detectorを起動する．

`topic`はrelayしているので，実機と同じファイルで大丈夫です．

```bash
roslaunch target_object_detector target_object_detector.launch
```

### 5. navigaterを起動する．

こちらはwaypointファイルが異なるので，実機とは別ファイルです．

```bash
roslaunch waypoint_navigator waypoint_navigator_gazebo.launch 
```
