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

---

Tsukuba Challenge 2015のときと違い、ロボット毎にリポジトリを分けているので、どういう運用にするか、まだ決まっていません。決まり次第、書きます。  
