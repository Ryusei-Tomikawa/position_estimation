# position_estimation
このパッケージはアルコマーカーを認識してロボットの位置補正を行うコードです

このコードはseed_noidで試しているため、worldファイルは注意が必要です（他のロボットを使用する際はseed_noid部分を消すこと）

launchファイルを起動すれば、アルコマーカー認識ノードと位置補正ノードが起動されるので、シミュレーションや実機環境、2DMapがあれば一応動くはず。。。

位置補正の終了部分のところはてきとーかつ、cmd_velの値も適宜調整する必要あり

# gazeboにAruCoMarkerが使えない！？
このリポジトリのlaunchを立ち上げてもAruCoMarker部分が真っ白であった場合以下の手順を踏む必要がある
原因：~/.gazebo/modelsの中にAruCoMarkerのモデルが入っていないため
ここのURLのリポジトリからcloneするとよい
[MOTOMAN-GP8Moveit!実機動作方法](http://www1.meijo-u.ac.jp/~kohara/cms/technicalreport/ros_motoman_gp8_setup)


  '''
