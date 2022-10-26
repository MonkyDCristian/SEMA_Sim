# Change simulator configuration

Whenever you want to start a simulation, you can customize the widgets that are available to optimize the simulation environment, for example, if you want to develop trajectory algorithms, you don't need to waste computing power simulating the depth perception view of the camera model Oak-D. In the same way, if you only want to test the vision system, you may not need the vacuum gripper.

**Note:** This simulator, as a university project, was developed with the aim of being multipurpose, so it can be reconfigured to be applied in other automation or research projects.

To start the simulation you can launch
```
roslaunch sema_gzsim sema_gzsim.launch
```
to start the simulation in gazebo or use
```
roslaunch sema_gzsim sema_gzsim_moveit.launch 
```
to start it using Moveit! motion planner.

In any of this two launch you can add the parameters:

* conveyor_belt_enabled:=true/false

* oak_d_enabled:=true/false     

* vgc10_enabled:=true/false  

By default all parameters are set to true.

### Example
```
roslaunch sema_gzsim sema_gzsim.launch conveyor_belt_enabled:=false oak_d_enabled:=false vgc10_enabled:=false  
```
**Note:** It will appear errors with the VGC10 controller and the conveyor belt controller, you can just ignore it or fix it by editing the **robot_ctrl** argument in the launch file, more information inside the launch file.
