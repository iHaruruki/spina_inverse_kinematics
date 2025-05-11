# my_spine_robot
### Node and Topic
![](rosgraph.png)
## Dependency

## Setup
```bash
$ cd ~/ros2_ws/src  #Go to ros workspace
$ git clone https://github.com/iHaruruki/spina_inverse_kinematics.git #clone this package
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source install/setup.bash
```
## Usage
```bash
1. $ ros2 launch spina_inverse_kinematics spina_robot_launch.py
# Use the ROS2 CLI to publish to the topic `/target_pose`
2. $  ros2 topic pub /target_pose geometry_msgs/PoseStamped '{
        header: { frame_id: "base_link" },
        pose: {
          position: { x: 1.2, y: 0.5, z: 0.8 },
          orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
        }
      }' --once
```
### Description
* header.frame_id: "base_link": このポーズが属する座標フレームを`base_link`に設定
* pose.position: エンドエフェクタ（手先）の目標位置を x=1.2 m, y=0.5 m, z=0.8 m で指定
* pose.orientation: エンドエフェクタの姿勢をクォータニオンで指定（ここでは回転無し＝単位クォータニオン）
## URDF test
```
$ ros2 launch urdf_tutorial display.launch.py model:=/home/haaruki/ros2_ws/src/spina_inverse_kinematics/urdf/spina_robot.urdf
```
## License
## Authors
## References
