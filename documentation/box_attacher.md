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

## Spawn boxes by /box_spawner_act_srv
