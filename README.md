# my_spine_robot
### Node and Topic
![](rosgraph.png)
## Dependency

## Setup
```bash
$ cd ~/ros2_ws/src  #Go to ros workspace
$ git clone  #clone this package
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source install/setup.bash
```
## Usage
```bash
1. $ ros2 launch spina_inverse_kinematics spina_robot_launch.py
#
2. $  ros2 topic pub /target_pose geometry_msgs/PoseStamped '{
        header: { frame_id: "base_link" },
        pose: {
          position: { x: 1.2, y: 0.5, z: 0.8 },
          orientation: { x: 1.0, y: 3.0, z: 3.0, w: 1.0 }
        }
      }' --once
```
## License
## Authors
## References
