# Conveyor Belt Control
For this project it was necessary to develop a recursive conveyor belt model, the most efficient way to do it was to create a cylindrical platform that moved the objects on it, in this case boxes, making it rotate with respect to its axis.

In the simulation, it is possible to control the movement of the conveyor belt by setting the angular speed in rad/sec.

**Note:** A PID joint velocity controller provided by the [ros_control](http://wiki.ros.org/ros_control) library is applied.

## Control the conveyor belt by topic 
First start the simuation with the UR10 and the conveyor belt, open a terminal and run:
```
roslaunch sema_gzsim sema_gzsim.launch oak_d_enabled:=false vgc10_enabled:=false  
```
Now set the velocity for the conveyor belt by publish the ros command:
```
rostopic pub /cb_joint_vel_controller/command std_msgs/Float64 "data: 0.1"
```
the conveyor should start to move at 0.1 rad/sec. You can insert an object into the simulation and place it on the conveyor to get a better view of the result.

## Control the conveyor belt by python code 

For this, and other SEMA simulation tools, it was developed a python class that you can copy and modified or just import it to your python file to easily use it. You can access to the conveyor controller code with the next path:

[~/SEMA_Sim/sema_ws/src/sema_gzsim/src/sema_gzsim/conveyor_belt_vel_ctrl.py](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/sema_ws/src/sema_gzsim/src/sema_gzsim/conveyor_belt_vel_ctrl.py)

### Use example
```
#!/usr/bin/env python3

import rospy
from sema_gzsim.conveyor_belt_vel_ctrl import ConveyorBeltVelocityCtrl

rospy.init_node("conveyor_belt_velocity_ctrl")
cb_vel_ctrl = ConveyorBeltVelocityCtrl()
	
vel = 0.1 #rad/s
cb_vel_ctrl.run(vel)
```

### Test it

**T1:**
```
roslaunch sema_gzsim sema_gzsim.launch oak_d_enabled:=false vgc10_enabled:=false  
```
**T2:**
```
rosrun sema_gzsim conveyor_belt_vel_ctrl.py
```

**Note:** For physic stability, don't set the angular velocity higher than 0.1 rad/sec.

## Next Tutorial 
[Spawn a pallet platform in gazebo.](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/documentation/pallet_spawner.md)
