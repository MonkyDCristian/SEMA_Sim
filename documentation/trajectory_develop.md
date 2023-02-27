# Develop and test a trajectory algorithm.

To move a box within the simulation using the UR10 we need to follow three steps. First, we need to link the box to the end effector, the link needs to be reflected in Moviet! scene. Second, we must plan and execute a trajectory to the destination position. Third, we must take off the box in Gazebo and Moveit!.

Sounds simple, right? We could use MoveGroupPythonInterface functions for this task, but we risk complicating our code. Better divide and conquer, which is why the [CompliteAttacher](https://github.com/MonkyDCristian/SEMA_Sim/blob/ROS-focus-develop/sema_ws/src/sema_moveit/src/sema_moveit/complite_attacher.py) and [Planner](https://github.com/MonkyDCristian/SEMA_Sim/blob/ROS-focus-develop/sema_ws/src/sema_moveit/src/sema_moveit/planner.py) classes were created.

Complite_attachque takes care of attaching and detaching a box, including it inside moveit! scene. Planner facilitates the parameterization of the planning and execution of trajectories. 

**Note:** check CompliteAttacher and Planner code for more information about how their work.

## Use example
```
#!/usr/bin/env python3

import rospy, numpy as np

from sema_gzsim.box_spawner import boxes_prms
from sema_gzsim.pose_compilation.pick_box_poses import pick_box_poses
from sema_gzsim.packing_examples import PackingExample1, PackingExample2, PackingExample3, PackingExample4

from sema_moveit.move_group_python_interface import MoveGroupPythonInterface
from sema_moveit.complite_attacher import CompliteAttacher
from sema_moveit.planner import Planner


class TrajectoryDemo(object):
	
	def __init__(self):
		self.packing = PackingExample1()
		self.packing.palet_prms = {"x":0.1, "y":0.75, "z":0.65, "yaw":0.0, "size":{"x":0.6, "y":0.9, "z":0.14}}
		self.packing.target = {"model":"m", "id":0, "x":0.0, "y":0.0, "z":0.0, "yaw":np.pi, "static":True}
		self.packing.box_prms = boxes_prms[self.packing.target["model"]]
		self.packing.offset = 0.004

		self.mgpi = MoveGroupPythonInterface()
		self.mgpi.show_variable()

		self.complite_attacher = CompliteAttacher(self.mgpi)
		self.complite_attacher.box_attacher.attach_params["ideal_attach"] =  True

		self.planner = Planner(self.mgpi)
		self.planner.joint_poses = pick_box_poses
		self.planner.setup()


	def run(self, dict_box):
		
		self.planner.go_to_joint_pose("pick_pose")
		
		if dict_box["id"] == 0:
			self.packing.set_first_box_pose()
		
		else:
			self.packing.target["id"] = count
			self.packing.get_next_pose()

		self.complite_attacher.attach_box(dict_box)
		self.planner.go_to_target(self.packing.target)
		self.complite_attacher.detach_box(dict_box)
		


rospy.init_node("trajectory_demo")
trajectory_demo = TrajectoryDemo()
dict_box = {"model":"m", "id": 0, "x":0.0, "y":0.0, "z":0.0, "yaw":0.0, "static":True}

for count in range(9):
    dict_box["id"] = count
    trajectory_demo.run(dict_box)

trajectory_demo.planner.go_to_joint_pose("pick_pose")
```

## Test it
**T1:**
```
roslaunch sema_gzsim setup_sim_trajectory.launch
```
**T2**
```
rosrun sema_gzsim trajectory_demo.py 
```

## Next Tutorial 
[Use a very simple image processing algorithm to detect box with the camera.](https://github.com/MonkyDCristian/SEMA_Sim/blob/ROS-focus-develop/documentation/box_detector.md)
