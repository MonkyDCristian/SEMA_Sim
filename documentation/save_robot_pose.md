# Save Robot Pose
**Note:** up to this point, you have probably already learned how to control the UR10 by rqt_joint_trajetory_controller or by Movit! interface with Rviz, anyway, if you haven't, I highly recommend you to check this quick start [Moveit! tutorial](https://ros-planning.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html), to learn the basic of this motion planning program.

Working with motion planners in robotic arms usually has some problems, such as [singularity](https://www.mecademic.com/en/what-are-singularities-in-a-six-axis-robot-arm) poses, in simple words, poses where the robot gets stuck. 

A common technique to avoid these conflicting poses is to have a set of safe poses, these are poses the robot can take that are far from the singularity zones and make it easier to apply motion planners.

There are others cases where having a set of poses can be useful, for example when you are setting up a repetitive motion or when you want the robot to start in some specifics pose to do an experiment.

For all these reason, the URPoseRegister class was created, this program allow you to save the actual pose data of the robot as a [dictionary](https://www.w3schools.com/python/python_dictionaries.asp) inside a python file with he next structure:
 ```
 example_directory =
  {'pose0': {'ur_joints': {'sema/elbow_joint': elbow_ang,
                           'sema/shoulder_lift_joint': shoulder_lift_ang, 
                           'sema/shoulder_pan_joint': shoulder_pan_ang, 
                           'sema/wrist_1_joint': wrist_1_ang, 
                           'sema/wrist_2_joint': wrist_2_ang, 
                           'sema/wrist_3_joint': wrist_3_ang}, 
             'eef_pose': {'position': {'x': eef_x_position, 
                                       'y': eef_y_position, 
                                       'z': eef_z_position}, 
                          'orientation': {'x': eef_x_orientation, 
                                          'y': eef_y_orientation, 
                                          'z': eef_z_orientation, 
                                          'w': eef_w_orientation}}}, 
  'pose1':...
  ...
  }}}}
 ```
 
[~/SEMA_Sim/sema_ws/src/sema_gzsim/node/ur_pose_register.py](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/sema_ws/src/sema_gzsim/node/ur_pose_register.py)

This program run along with the simulation, and you can operate by using the rqt_reconfing GUI tool.

## Test it
T1:
```
roslaunch sema_gzsim sema_gzsim_moveit.launch oak_d_enabled:=false        
```
T2:
```
rosrun rqt_reconfigure rqt_reconfigure       
```
Now go to "ur_pose_register" inside the rqt_reconfigure gui and write the name of the python file where you want to save the poses, by default is **example.py**. Then write the name of the pose, by default is **pose0**, when you are ready click on the enable box and the poses will be registered in the next path of your sema_gzsim pkg:

[~/SEMA_Sim/sema_ws/src/sema_gzsim/node/pose_compilation](https://github.com/MonkyDCristian/SEMA_Sim/tree/main/sema_ws/src/sema_gzsim/node/pose_compilation)

Any time you change the pose name and press enter, a new pose will be registered. Be careful not to overwrite a pose by pressing enter. You can also change the name of the file and create another pose collection. 

## Next Tutorial

[Control the robot by Moveit! with Python3.](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/documentation/moveit.md)
