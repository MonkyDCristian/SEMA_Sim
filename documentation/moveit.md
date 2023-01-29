# Control the robot by Moveit! with Python3
Note: Congratulations! You have arrived to the last tutorial of this first SEMA simulator version, put this [theme](https://www.youtube.com/watch?v=DsemU6yb7hA).

The time has come to develop code that actually allows us to automate the palletizing process.
As a development project,we are using Python3 as our prototyping language. C++ is the original Moveit! develop language but, conveniently, there's a [Move Group Python Interface](https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html) that allows us to use Moveit! in the same way that we could with C++. Click on the link above and follow the tutorial to get a basic understanding of what you can do with this amazing interface.

To easily use this interface, the [MoveGroupPythonInterface](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/sema_ws/src/sema_moveit/src/sema_moveit/move_group_python_interface.py) class was created, it is a shortened version of [MoveGroupPythonInterfaceTutorial](https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py) for directly applying  and saving a lot of redundancy code. It has the same function as the tutorial version and can be called from the sema_moveit pkg.

## Use example
```
#!/usr/bin/env python3

import rospy
from sema_moveit.move_group_python_interface import MoveGroupPythonInterface
mgpi = MoveGroupPythonInterface()
```

## Use registered poses

Image that we already have registered a compilation of pose, now we want the robot to change from one pose to another. To do that we can executive a sequence of poses using the MoveGroupPythonInterface:

```
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

from sema_moveit.move_group_python_interface import MoveGroupPythonInterface
from pose_compilation.pick_box_poses import pick_box_poses


def tf2joint_state_msg(lib_poses, pose_name):
	goal_joints = JointState()
	
	goal_joints.name = list(lib_poses[ pose_name]["ur_joints"].keys())
	goal_joints.position = list(lib_poses[ pose_name]["ur_joints"].values())

	return goal_joints
		
mgpi = MoveGroupPythonInterface()
mgpi.show_variable()

goal_joints_0 = tf2joint_state_msg(pick_box_poses, "pick_pose")
goal_joints_1 = tf2joint_state_msg(pick_box_poses, "middle_place_pose")
goal_joints_2 = tf2joint_state_msg(pick_box_poses, "final_place_pose")

mgpi.move_group.go(joints=goal_joints_0 , wait=True)
mgpi.move_group.go(joints=goal_joints_1 , wait=True)
mgpi.move_group.go(joints=goal_joints_2 , wait=True)
```
**Note:** the tf2joint_state_msg function transform the registered joints pose to [JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html) msg, necessary for setting with the move group program.

## Test it
**T1:**
```
roslaunch sema_gzsim sema_gzsim_moveit.launch oak_d_enabled:=false
```
**T2**
```
rosrun sema_gzsim mgpi_test.py
```

## Next Tutorial 
[Adding objects to Moveit! planning scene.](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/documentation/obj2scene.md)


