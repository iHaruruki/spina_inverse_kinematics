# my_spine_robot

ROS2パッケージ for 6モジュール型スパインロボットのIKソルバとURDF

## ビルド
```bash
colcon build --packages-select my_spine_robot
```

## 実行
1. URDFを見る（rvizなどで表示）
2. ノードを起動
```bash
ros2 launch my_spine_robot spine_robot_launch.py
```
3. 目標姿勢を送信
```bash
ros2 topic pub /target_pose geometry_msgs/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 2.0, y: 2.0, z: 1.0}, orientation: {w: 1.0}}}"
```
