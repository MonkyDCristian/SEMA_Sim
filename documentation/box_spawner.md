# Box Spawner

The simulation features 5 box models, all with the same appearance but with different sizes and mass values.

![Alt text](/imgs/box_models.png)

|Name          |Abbreviation| Mass (kg) | Size x (mt) | Size y (mt) | Size z (mt) |
|--------------|------------|------|-----------|-----------|-----------|
|little        |l           |0.5   |   0.12    |     0.15  |   0.12    |        
|middle_little |ml          |0.84  |   0.16    |     0.19   |   0.12   |
|middle        |m           |2.08  |   0.2     |      0.3  |   0.15    |
|big_middle    |bm          |6.94  |   0.3     |      0.4  |   0.25    |
|big_box       |b           |13.89 |   0.5     |      0.8  |   0.4     |

**Note**: The maximum piload that the UR10 can move with a VGC10 is around 7 to 8 kg, so you shouldn't be available to move the big box model in real life. Anyway, it is there if you want to experiment with big boxes, just don't try it with a real robot.

BoxSpawnerActSrv is the python class that allows you to spawn a sequence of boxes with respect to global position and orientation, just like PalletSpawner, but in this case the boxes are not static. This class was developed using the [ROS action](http://wiki.ros.org/actionlib), so it does not interfere with the execution of the main program.

[~/SEMA_Sim/sema_ws/src/sema_gzsim/src/sema_gzsim/box_spawner_act_srv.py](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/sema_ws/src/sema_gzsim/src/sema_gzsim/box_spawner_act_clt.py)

## Spawn boxes by topic 
You can publish a BoxSpawnerActionGoal message to the /box_spawner_act_srv/goal to spawn a box topic. In this message, you must configure: the sequence using the abbreviation box name, the frequency at which the boxes are generated, using the hz parameter, and the box pose.

### Test it
**T1:**
```
roslaunch sema_gzsim sema_gzsim.launch oak_d_enabled:=false vgc10_enabled:=false  
```
**T2:**
```
rosrun sema_gzsim conveyor_belt_vel_ctrl.py
```
**T3:**
```
rostopic pub /box_spawner_act_srv/goal sema_gzsim/BoxSpawnerActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal: {sequence: 'l,ml,m,bm,b', hz: 0.2, x: -0.6, y: -0.3, z: 0.8, yaw: 0.0}" 
```

## Spawn boxes by box_spawner_act_clt

As we are working with a ROS action service, we need a ROS action client to interact with it by code. The  BoxSpawnerActClt is the class that you can use for this purpose. 

[~/SEMA_Sim/sema_ws/src/sema_gzsim/src/sema_gzsim/box_spawner_act_clt.py](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/sema_ws/src/sema_gzsim/src/sema_gzsim/box_spawner_act_clt.py)

### Use example
```
#!/usr/bin/env python3

import rospy
from sema_gzsim.box_spawner_act_clt import BoxSpawnerActClt

rospy.init_node("box_spawner_act_clt")
box_spawner = BoxSpawnerActClt()

box_sequence = ["ml", "m", "ml"]
spawn_params = {"sequence":"", "hz":1.0, "x":-0.6, "y":-0.3, "z":0.8, "yaw":0.0}

for box_type in box_sequence:
  spawn_params["sequence"] = box_type

  box_spawner.set_params(spawn_params)
  box_spawner.run()

  spawn_params["y"] += 0.5
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
**T3:**
```
rosrun sema_gzsim box_spawner_act_clt.py
```

**Note:** Now it is the perfect time for you to overview the [SEMA demo code](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/sema_ws/src/sema_gzsim/node/sema_demo.py), that combines conveyor control, pallet spawner and box spawner.

## Next Tutorial
[Create an ideal or a relative attach between the vacuum gripper and a box.](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/documentation/box_attacher.md)
