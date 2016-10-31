# Tsukuba Challenge 2016
つくばチャレンジ2016用のリポジトリです。  
GitHub-Flowによる運用を行いたいと思います。  

## How to use ?
1. `catkin_workspace`の作成  
好きなディレクトリで`catkin_workspace`を作ってください。ここでは分かりやすいように`tc2016_ws`とします。特別な理由が無い限りこのディレクトリ名を推奨します。  
```bash
cd <your favorite directory to develop>
mkdir -p tc2016_ws/src
cd tc2016_ws/src
catkin_init_workspace
cd ..
catkin_make
```
`catkin_make`が成功することを確かめましょう。  

2. 本リポジトリのクローン  
本リポジトリを先ほど作ったワークスペースの`src`にクローンしましょう。  
```bash
cd tc2016_ws/src
git clone git@github.com:CIR-KIT/TC2016.git
```
チームメンバーであれば``ssh``でクローンできるので、もし失敗する場合には`ssh`の設定を見直すか、チームメンバーに参加できているかを確認してください。  

3. 他のパッケージのダウンロード
`wstool`を使って必要なパッケージをダウンロードしてきます。
```bash
cd tc2016_ws/
wstool init src src/TC2016_for_thirdrobot/third_robot.rosinstall
```
他のパッケージのアップデートは
```bash
cd tc2016_ws
wstool update -t src
```
でできます。

.rosinstallファイルに追記した場合は
```
cd tc2016_ws
wstool merge -t src src/TC2016_for_thirdrobot/third_robot.rosinstall
```
とします。

### 自律移動の流れ
ゼロの状態から自律移動を行うまでの流れを説明します。

1. 地図の作成  
環境中をロボットを走らせながら`gmapping`等を使って地図を作ります。リアルタイムに作ってもいいし、`rosbag`で保存してからやってもいいです。個人的にはパラメータを変えながら試せるので`rosbag`に記録してからのほうがいいと思います。以下`rosbag`に記録してから行うことを想定します。地図作成以外にも使うので`rosbag record -a`で全部保存しましょう。`rosbag`から地図を作る場合には`gmapping`の場合、  
```bash
rosrun gmapping slam_gmapping_replay --scan_topic=/scan_multi --bag_filename=2016-10-19-15-49-19.bag  _particles:=50 _delta:=0.1 _iterations:=30 _str:=15.5 _stt:=15.5 _linearUpdate:=0.3 _angularUpdata:=0.1
```
こんな感じで実行できます。トピック名、bagファイル名、パラメータは適宜変更してください。パラメータに関しては実際のところちゃんとオドメトリがとれていれば特にデフォルトで綺麗にできます。  
`rviz`上でいい感じに地図が出来たら、地図を保存します。
```
rosrun map_server map_saver -f filename
```
カレントディレクトリに`-f`オプションで指定した名前の地図が出来ます。`pgm`ファイルが実態ですが、地図の情報が含まれている`yaml`ファイルも重要です。地図の名前を変更したいときはファイル名自体を変更することに加えて、`yaml`ファイル内の画像ファイルの名前も変更しないといけません。  

2. Waypointの作成  
今回は`ros_waypoint_generator`を使います。`catkin workspace`にクローンしましょう。ちゃんと`wstool`でやればすでにあるかもしれません。
```bash
git clone https://github.com/AriYu/ros_waypoint_generator.git
```
`catkin_make`をしてコンパイル後、起動します。
```bash
rosrun ros_waypoint_generator ros_waypoint_generator
```
次に、`map_server`を立ち上げて、地図を配信します。
```bash
rosrun map_server map_server map.yaml
```
`map.yaml`には1.で作った地図のファイルパスを指定します。  
次に`amcl`を走らせます。
```bash
roslaunch third_robot_2dnav amcl_diff.launch
```
次に`bag`ファイルを再生し、自己位置推定を開始します。`--clock`オプションは指定しないでください。
```
rosbag play example.bag
```
ここが難しいですが、`amcl`の初期位置が`rosbag`を開始しないと指定出来ないため、`rosbag play`を始めたらすぐに頑張って初期位置合わせをしてください。  
自己位置推定が始まるとwaypointが生成されていきます。`rviz`で`interactive marker`を表示するとwaypointを見ることができます。waypointの姿勢は`tf`を表示してください。また、markerはマウスのドラッグで移動出来ます。特に最初はどうしても思っているのと違うところにwaypointが出来てしまうので移動してあげてください。他にも`rviz`上部にある`Publish point`を使ってもwaypointを追加できます。ただし、現状ではwaypoint最後にしか追加出来ないので間に追加したかったら保存後に自分で編集してください。このへんを便利で出来そうなプルリクを待ってます。  
また、どれだけ移動したらwaypointを追加するかとかもパラメータで変えられます。  
waypointを作り終えたらwaypointを保存します。
```bash
rosrun ros_waypoint_generator ros_waypoint_saver
```
カレントディレクトリに次の形式のcsvファイルが生成されます。  
```
x, y, z, qx, qy, qz, qw, is_searching_area, reach_threshold
```
`is_searching_area`は探索エリアかどうかを判定するフラグで`1`なら探索エリアです。表計算ソフトで開くと一気に編集可能なのでこの辺という感じで一気に`1`にしましょう。`reach_threshold`はそのwaypointにどれだけ近づいたらゴールとするかです。直径なので例えば`3`を指定していたらwaypointから`1.5`[m]以内に入ればゴールになります。  
waypointを確認したいときは
```bash
rosrun ros_waypoin_generator ros_waypoint_generator --load waypoints.csv
```
でロードできます。探索エリアが指定されていればmarkerの色が黄色になります。  

ここまで準備は終わりです。  

3. Navigation  
2.で作ったwaypointファイルを`waypoint_navigator/waypoints`にコピーしておきましょう。本当はどこでもいいです。`waypoint_navigator/launch/waypoint_nagigator.launch`の`waypointsfile`をさっき作ったやつに書き換えます。  

実際にナビゲーションしましょう。実機なら
```bash
roslaunch third_robot_2dnav autorun.launch
```
でロボットドライバ他`move_base`などを立ち上げます。
navigatorをlaunchします。
```bash
roslaunch waypoint_navigator waypoint_navigator.launch
```
ロボットが自律移動を始めます。  

4. 探索対象をみつける  
- 実機の場合：  
```
roslaunch target_object_detector target_object_detector.launch
```
- 実際に実験出来ない時：  
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
