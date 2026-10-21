ROS 2 Jazzy / Gazebo Harmonic 四辺柵・事前強化学習デモ

前提: ros2_gazebo_gui コンテナ、/root/pai_ws/src/my_robot_control の model.sdf と bridge.yaml が存在すること。

1. WindowsからWSL Ubuntuを開く。
2. WSL Ubuntuで実行:
   docker start ros2_gazebo_gui
   docker cp /mnt/c/Users/otani/Downloads/gazebo_fence_rl ros2_gazebo_gui:/root/
   docker exec -it ros2_gazebo_gui bash
   
3. Docker内で実行:
   source /opt/ros/jazzy/setup.bash
   apt-get update && apt-get install -y python3-numpy ros-jazzy-ros-gz
   cd /root/gazebo_fence_rl
   bash run_demo.sh

注意:
- 既存のGazeboサーバー / robot.launch.py と teleop_twist_keyboard を終了してから起動する。
- Q-learningは既知の正方形環境の2D運動モデルで事前学習する。Gazebo上での実機学習ではない。
- 柵を含むワールドと安全領域は固定。柵の中心線 +/-3.0m、ロボット中心の安全境界 +/-2.35m。
- ROS /odom の原点がロボット初期位置と一致する前提。
- ROSモデルの駆動プラグイン、車輪サイズなどによっては調整が必要。
- GUIにワールドが出ない場合は gz sim -g のログと Gazebo Transport を確認。
- Ctrl+Cでデモを停止する。
