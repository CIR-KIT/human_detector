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
