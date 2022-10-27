# Pallet Spawner
To make it easy to customize your packaging environment, the python 3 class **Pallet Spawner** was developed. This class allows you to place a horizontal pallet wherever you want with respect to the global position and orientation.

[~/SEMA_Sim/sema_ws/src/sema_gzsim/src/sema_gzsim/pallet_spawner.py](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/sema_ws/src/sema_gzsim/src/sema_gzsim/pallet_spawner.py)

![Alt text](/imgs/pallet_spawner.png)

## Code example
```
#!/usr/bin/env python3

import rospy
from sema_gzsim.pallet_spawner import PalletSpawner

rospy.init_node("pallet_spawner")
spawn_params = {"x":0.3, "y":0.8, "z":0.1, "yaw":0.0}

pallet_spawner = PalletSpawner()
pallet_spawner.set_params(spawn_params)
pallet_spawner.run()
```

To test it you can run:

**T1:**
```
roslaunch sema_gzsim sema_gzsim.launch oak_d_enabled:=false vgc10_enabled:=false  
```
**T2:**
```
rosrun sema_gzsim pallet_spawner.py
```
**Note:** Check the Gazebo [SpawnModel service](http://docs.ros.org/en/electric/api/gazebo/html/srv/SpawnModel.html) if you want to add spawn parameters.

### Next tutorial 
[spawn a box, or a sequence of boxes, in the simulation.]( https://github.com/MonkyDCristian/SEMA_Sim/blob/main/documentation/box_spawner.md)
