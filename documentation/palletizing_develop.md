#  Use a very simple image processing algorithm to detect box with the camera.

Congratulations, you have reached the final tutorial, but I have to tell you something, I lied to you, this is not a tutorial, it is a training area. In other words, the one who is going to develop the complete palletizing cycle is going to be you, using all the classes presented in the last tutorials.

Now create your palletiziting_demo.py file in the sema_gzsim/node/beta directory, copy-paste the following code and complete it:

```
#!/usr/bin/env python3

import rospy, numpy as np

from sema_gzsim.packing_examples import PackingExample1, PackingExample2, PackingExample3, PackingExample4
from sema_gzsim.box_spawner import boxes_prms

from sema_moveit.move_group_python_interface import MoveGroupPythonInterface
from sema_moveit.complite_attacher import CompliteAttacher
from sema_moveit.planner import Planner

from sema_beta_poses import sema_beta_poses

from std_msgs.msg import Empty


class BetaDemo(object):
	
	def __init__(self):
		self.variables_init()
		self.connections_init()
		

	def variables_init(self):
		self.packing = PackingExample4()
		self.packing.palet_prms = {"x":0.1, "y":0.75, "z":0.65, "yaw":0.0, "size":{"x":0.6, "y":0.9, "z":0.14}}
		self.packing.target = {"model":"m", "id":0, "x":0.0, "y":0.0, "z":0.0, "yaw":np.pi, "static":True}
		self.packing.box_prms = boxes_prms[self.packing.target["model"]] # set  the size of the middle boxes
		self.packing.offset = 0.004

		self.mgpi = MoveGroupPythonInterface()
		self.mgpi.show_variable()

		self.complite_attacher = CompliteAttacher(self.mgpi)
		self.complite_attacher.box_attacher.attach_params["ideal_attach"] =  False

		self.planner = Planner(self.mgpi)
		self.planner.joint_poses = sema_beta_poses
		self.planner.setup(replanning=True)
	

	def connections_init(self):
		# complete code here
        pass

	def callback(self, msg):
		# complete code here
        pass

	def pick_box(self):
		# complete code here
        pass


	def move_box_to_target(self):
		# complete code here
        pass
		

if __name__ == "__main__":
	rospy.init_node("beta_demo")
	beta_demo = BetaDemo()
	beta_demo.dict_box = {"model":"m", "id": False, "x":0.0, "y":0.0, "z":0.0, "yaw":0.0, "static":True}
    
    # go to vision position
	beta_demo.planner.go_to_joint_pose("vision")
	
    rospy.spin()
```

Tips:
1.- You can use the setup_sim_vison.py as a palletizing environment.

2.- The file [sema_beta_poses.py]() has 2 registered poses: vision and pick. The "vision" pose is to position the camera to see the incoming boxes and the "pick" pose is to grab the boxes that are in the pick up zone.

3.- Just to help a bit, you could create an algorithm using the following steps.

1. Put the robot in "vision" position.

2. Wait for the box_in_position signal to arrive from the box_detector node.

3. Move the robot to the "pick up" position

4. Attach the box

5. Return to viewing position

6. Get the destination position of the box with a packing algorithm.

7. Generate and execute a trajectory to the target position.

8. Separate the box.

9. active the box_detector node by a topic msg.

10. Go back to step 1.


If you feel lost or just want to get a visual idea of ​​how the algorithm should work.
you could run the following two commands

**T1:**
```
roslaunch sema_gzsim setup_sim_beta.launch
```
**T2:**
```
rosrun sema_gzsim beta_demo.py
```

**Note:** the [beta_demo.py]() is the algorithm you are developing, try not to check it unless you really need it.



