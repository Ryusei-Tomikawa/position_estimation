# position_estimation
このパッケージはTHK(株)が開発したSeed-Noidを用いて、AruCoMarkerを用いたロボットの位置補正を行うコードです

launchファイルを起動すれば、アルコマーカー認識ノードと位置補正ノードが起動されるので、シミュレーションや実機環境、2DMapがあれば一応動くはず。。。

位置補正の終了部分のところはてきとーかつ、cmd_velの値も適宜調整する必要あり

# 使い方
事前にseed-noidの環境構築やこのリポジトリのworldのGazebo環境の地図生成を行っておくこと.

[Seed-Noidの環境構築](https://github.com/seed-solutions/seed_r7_ros_pkg)

・seed_r7_gazeboを立ち上げる際は、このリポジトリのworldファイルを読み込むようにlaunchを変更しておくこと

・wheel_with_dummy.launchの部分も地図のyamlをこの環境に合わせたものに変更すること

  ```shell
  $ roslauch seed_r7_gazebo seed_r7_example.launch
  $ roslaunch seed_r7_navigation wheel_with_dummy.launch
  $ roslaunch position_estimation estimation.launch use_lauguage:=true
  ```
