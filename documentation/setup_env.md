# Create a setup environmental file in python

[~/SEMA_Sim/sema_ws/src/]()

## Use example
```
#!/usr/bin/env python3

```

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
[Fast palletizing by teleportation and static positioning of boxes.](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/documentation/box_teleport.md)
