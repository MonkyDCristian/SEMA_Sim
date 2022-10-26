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

BoxSpawnerActSrv is the python class that allows you to spawn a sequence of boxes with respect to global position and orientation, just like PalletSpawner, but in this case the boxes are not static. This class was developed using the ROS action, so it does not interfere with the execution of the main program.

[~/SEMA_Sim/sema_ws/src/sema_gzsim/node/box_spawner_act_srv.py](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/sema_ws/src/sema_gzsim/node/box_spawner_act_srv.py)

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
