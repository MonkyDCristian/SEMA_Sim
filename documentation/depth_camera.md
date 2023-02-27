# Setup and use a simulated depth camera.

![Alt text](/imgs/depth_perception.png)

Congratulations, you've reached the last of the basic level tutorials. Recalling what you have learned so far, we can say that you already know how to move the robot using a GUI, how to include boxes and pallets in the simulation environment, how to grab boxes, and how to record the robot's poses by measuring value of the joints.

In this tutorial we will learn how to use a [depth camera](https://thesweetcamera.com/what-is-depth-camera/) through a [Gazebo plugin](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins), the missing piece in the system that will allow us to detect incoming boxes and perform a palletizing mission using manual (non-autonomous) human control.

## Why do we need a depth camera?
First of all let's think about why we really need a depth camera, obviously it can be used to detect a dummy box and get its position through some kind of complicated algorithm, but actually we can access to each position and orientation object, in real time, by subscribing to the [Gazebo](http://wiki.ros.org/gazebo) topic ["gazebo/model_states"](http://docs.ros.org/en/api/gazebo_msgs/html/msg/ModelStates.html) or request a specific object information via the ["gazebo/get_model_state"](http://docs.ros.org/en/api/gazebo_msgs/html/srv/GetModelState.html) service.

![Alt text](/imgs/display_of_boxes.png)

[~/SEMA_Sim/sema_ws/src/sema_gzsim/node/publish_markers.py](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/sema_ws/src/sema_gzsim/node/publish_markers.py)

**Note:** The "gazebo/model_states" topic is used to display the box frames in RVIZ via [ROS Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker).

Knowing that simulating the camera requires more processing power, why would we insist on including it? Why not just use the Gazebo interface?
Well, this is explained by the objective of the simulator, which is to serve as a complete development environment for a real palletizing system. We won't have the poses of the boxes given by gazebo in the field, so we won't be able to use them to develop the full palletizing flow.

**Note:** It is good for you to know that there are simple and low consumption ways to obtain the poses of the boxes. Think of the case where you are only interested in developing algorithms to transport boxes from one place to another, for this you don't have to spend computing power detecting the box by vision, you could access it through the "gazebo/get_model_state" service.

## Introduction
Let's start with this tutorial. The first thing is to consider what type of camera we are going to simulate, as mentioned before, the idea is that the system corresponds to reality. So if we are going to simulate a camera, it is most appropriate for us to have the same properties as the camera we plan to use in the field. [Luxonis](https://shop.luxonis.com/) brand have the OpenCV AI kits depth (OAK-D), models of cameras that support ROS and have a very good framework for developing computers vision algorithms, this is perfect for this project. Of these types of camera we will simulate the most recent model, the [OAK-D-PRO](https://shop.luxonis.com/collections/usb/products/oak-d-pro), we will integrate it and evaluate it in our virtual gazebo environment to know if it is suitable or not for this palletizing system. You will see that it is possible to set up a complete virtual simulation of the camera by following two simple steps

## Step 1: Configure the camera's Unified Robot Description Format (URDF) model.

To add a model to gazebo, we must have a [URDF](http://classic.gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros) file that describes its physical properties: appearance, collision space, and inertia.
The visualization of the camera can be obtained through a 3D model file, such as .dae or .stl. In this case we will obtain the models from the depthai_bridge ROS package, the collision space of this can be obtained from the [OAK-D-PRO documentation](https://docs.luxonis.com/projects/hardware/en/latest/pages/DM9098pro.html) while we can approximate its inertia using the [inertia equation](https://en.wikipedia.org/wiki/List_of_moments_of_inertia) corresponding to a cuboid.
To simplify this process we will describe the camera with [xacro](http://wiki.ros.org/xacro), a sintered file format to write URDF models.

Let create the file, we go to the urdf dictionary of the sema_description package, and create a file called oak_d_pro.xacro.

```
roscd sema_description/urdf & touch oak_d_pro.xacro
```

Inside the file paste the following code:
```
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="oak_d">
   
    <!-- OAK-D-PRO macro object model -->
    <xacro:macro name="oak_d_pro" params="camera_name parent prefix
                                        base_frame:=oak-d_frame camera_model:=OAK-D-PRO
                                        x:=0 y:=0 z:=0 R:=0 P:=0 Y:=0
                                        r:=0.8 g:=0.8 b:=0.8 a:=0.8"> <!-- color params -->
      
        <!-- OAK-D-PRO info: https://docs.luxonis.com/projects/hardware/en/latest/pages/DM9098pro.html -->

        <!-- physical parameters -->
        <xacro:property name="cam_mass"        value="0.091"  />
        <xacro:property name="cam_size_x"      value="0.097"  /> 
        <xacro:property name="cam_size_y"      value="0.0235" />
        <xacro:property name="cam_size_z"      value="0.0295" />
        <xacro:property name="cam_x_col"       value="-0.01"  />
        <xacro:property name="cam_y_col"       value="0.0"    />
        <xacro:property name="cam_z_col"       value="-0.005" />

        <!-- base_link of the sensor -->
        <link name="${prefix}${base_frame}">
        <visual>
                <origin xyz="0.0 0.0 0.0" rpy="${pi/2} 0.0  ${pi/2}"/>
                <geometry>
                    <mesh filename="package://depthai_bridge/urdf/models/${camera_model}.stl" />
                </geometry>
                
                <material name="mat">
                    <color rgba="${r} ${g} ${b} ${a}"/>
                </material>
            </visual>
            
            <collision name="${camera_name}_collision">
                <origin xyz="${cam_x_col} ${cam_y_col} ${cam_z_col}" rpy="0.0 0.0 ${pi/2}"/>
                <geometry>
                    <box size="${cam_size_x} ${cam_size_y} ${cam_size_z}"/>
                </geometry>
            </collision>
            
            <inertial>
                <origin xyz="${cam_x_col} ${cam_y_col} ${cam_z_col}" rpy="0.0 0.0 ${pi/2}"/>
                <mass value="${cam_mass}"/>
                <inertia
                    ixx="${cam_mass*(cam_size_y*cam_size_y+cam_size_z*cam_size_z)/12}"
                    iyy="${cam_mass*(cam_size_x*cam_size_x+cam_size_z*cam_size_z)/12}"
                    izz="${cam_mass*(cam_size_x*cam_size_x+cam_size_y*cam_size_y)/12}"
                    ixy="0"
                    ixz="0"
                    iyz="0"/>
            </inertial>
        </link>

        <joint name="${prefix}${camera_name}_center_joint" type="fixed">
            <parent link="${parent}"/>
            <child link="${prefix}${base_frame}"/>
            <origin xyz="${x} ${y} ${z}" rpy="${R} ${P} ${Y}" />
        </joint>

        <!-- RGB Camera optical frame  -->
        <link name="${prefix}${camera_name}_rgb_camera_optical_frame"/>

        <joint name="${prefix}${camera_name}_rgb_camera_optical_joint" type="fixed">
            <origin xyz="0 0 0" rpy="-${pi/2} 0 -${pi/2}"/>
            <parent link="${prefix}${base_frame}"/>
            <child link="${prefix}${camera_name}_rgb_camera_optical_frame"/>
        </joint>
    </xacro:macro>

    <xacro:arg name="camera_name"   default="my_cam"  />
    <xacro:arg name="parent_frame"  default="base_link"   />
    <xacro:arg name="prefix"        default=""            />
    
    <link name="$(arg parent_frame)"/>
    
    <xacro:oak_d_pro camera_name="$(arg camera_name)" parent = "$(arg parent_frame)" prefix="$(arg prefix)"/>

</robot>
```

Now you can check the model in Gazebo and RVIZ  running the next command on the terminal:
```
roslaunch sema_description spawm_model.launch name:=oak_d_pro
```
You can also check the collision space and the inertial by the view Gazebo setting: 

![Alt text](/imgs/urdf_cam_model.png)

## Step 2: Configure gazebo plugin for depth camera.

Once we have the physical representation of our camera, we only have to add the [gazebo openni_kinect plugin](https://classic.gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros) and configure its parameters.

Open the file oak_d_pro.xacro and add the following code after the RGB camera optical frame joint declaration, just before the line "</xacro:macro>".

```
<!-- OAK-D-PRO info: https://docs.luxonis.com/projects/hardware/en/latest/pages/DM9098pro.html -->
<!-- vision parameters -->
<xacro:property name="base_line"       value="0.075"/> <!-- m -->
<xacro:property name="h_fov"           value="1.25664"/> <!-- 72 deg -->
<xacro:property name="width"           value="1280"/> <!-- pix -->
<xacro:property name="height"          value="800"/> <!-- pix -->
<xacro:property name="min_depth_range" value="0.7"/> <!-- m -->
<xacro:property name="max_depth_range" value="15"/>  <!-- m -->

<!-- Gazebo camera setup -->
<gazebo reference="${prefix}${base_frame}">
    <sensor type="depth" name="${prefix}${camera_name}_sensor">
        <update_rate>18.0</update_rate>
    
        <camera name="${prefix}${camera_name}_camera">
            <horizontal_fov>${h_fov}</horizontal_fov>
            <image>
                <width>${5*width/8}</width>
                <height>${5*height/8}</height>
                <format>R8G8B8</format>
            </image>
            
            <clip>
                <near>0.065</near>
                <far>10.0</far>
            </clip>
            
            <noise>
                <type>gaussian</type>
                <!-- Noise is sampled independently per pixel on each frame.
                    That pixel's noise value is added to each of its color
                    channels, which at that point lie in the range [0,1]. -->
                <mean>0.0</mean>
                <stddev>0.007</stddev>
            </noise>
        </camera>
        
        <plugin name="depth_camera_plugin_${camera_name}" filename="libgazebo_ros_openni_kinect.so">  
            <!-- plugin config data: https://classic.gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros -->
            <baseline>${base_line}</baseline>
            <pointCloudCutoff>${min_depth_range}</pointCloudCutoff>
            <pointCloudCutoffMax>${max_depth_range}</pointCloudCutoffMax>
            <alwaysOn>true</alwaysOn>
            <updateRate>0.0</updateRate>
            <cameraName>camera_ir</cameraName>
            <imageTopicName>/camera/color/image_raw</imageTopicName>
            <cameraInfoTopicName>/camera/color/camera_info</cameraInfoTopicName>
            <depthImageTopicName>/camera/depth/image_raw</depthImageTopicName>
            <depthImageCameraInfoTopicName>/camera/depth/camera_info</depthImageCameraInfoTopicName>
            <pointCloudTopicName>/camera/depth/points</pointCloudTopicName>
            <frameName>${prefix}${camera_name}_rgb_camera_optical_frame</frameName>
            <distortionK1>0.00000001</distortionK1>
            <distortionK2>0.00000001</distortionK2>
            <distortionK3>0.00000001</distortionK3>
            <distortionT1>0.00000001</distortionT1>
            <distortionT2>0.00000001</distortionT2>
        </plugin>
    </sensor>
</gazebo>
```
**Note:** Here we have again use the same parameters of the oficial OAK-D-PRO documentation. Image size has been scaled to 5/8 to reduce processing weight. Some [distortion](https://classic.gazebosim.org/tutorials?tut=camera_distortion) and Gaussian noise have also been added to it, just to make it more realistic.

Finally, launch the model, go to RVIZ, add the depth camera display plugin and check how it works:
```
roslaunch sema_description spawm_model.launch name:=oak_d_pro
```
![Alt text](/imgs/gz_oak_d_pro.png)

## Cameras already available for SEMA sim

SEMA sim has three models of oak-d cameras, the LITE, POE and PRO. These models are written as a macro object model and you can call them from the depthai_camera_macro.xaro file in sema_descrition/urdf/macro.

Lines 134 to 148 of the sema_description/urdf/sema.xacro file show you how to set up the camera on the UR arm.

```
<!-- add Depth camera model and plugin  -->
  <xacro:if value="$(arg oak_d_enabled)">
    <xacro:arg name="camera_name"   default="oak" />
    <xacro:arg name="camera_model"  default="OAK-D-LITE" />
    <xacro:arg name="parent_frame"  default="$(arg prefix)wrist_3_link" />
    <xacro:arg name="cam_pos_x"     default="0.08" />
    <xacro:arg name="cam_roll"      default="-${pi}" />
    <xacro:arg name="cam_pitch"     default="-${pi/2}" />
  
    <xacro:include filename="$(find sema_description)/urdf/macro/depthai_camera_macro.xacro"/>
    <xacro:depthai_camera camera_name = "$(arg camera_name)" parent = "$(arg parent_frame)"
                          camera_model = "$(arg camera_model)" prefix = "$(arg prefix)"
                          x = "$(arg cam_pos_x)" R = "$(arg cam_roll)" P = "$(arg cam_pitch)"/>
  </xacro:if>
```

**Note:** The OAK-D-LITE model is the lightest, smallest and has the shortest stereo depth distance. This makes it a very suitable camera to attach to the end effector robotic arm and use it to detect close objects, for that reason we will use it for future tutorials. 

## Next Tutorial 
[Control the robot by Moveit! with Python3.](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/documentation/moveit.md)
