## Launch files

### [sim_launch.py](launch/sim_launch.py)

Launch ROV or AUV simulation in Gazebo.
Calls [bringup.py](launch/bringup.py).

To see parameters: `ros2 launch --show-args orca_bringup sim_launch.py`

The launch of ROV depends on the sub.parm.  
For BlueROV2, it relies on [sub.parm](#~/home/orca4/ardupilot/Tools/autotest/default_params/sub.parm)  
For BlueROV2 Heavy, it relies on [sub-6dof.parm](#~/home/orca4/ardupilot/Tools/autotest/default_params/sub-6dof.parm)

### [bringup.py](launch/bringup.py)

Bring up all core ROV and AUV nodes, including ORB_SLAM2 and Nav2.
Calls [navigation_launch.py](launch/navigation_launch.py).

### [navigation_launch.py](launch/navigation_launch.py)

Nav2 navigation launch file, modified to avoid launch the velocity smoother.

## Scenarios

### Full automation

In a terminal run:
~~~
source src/orca4/setup.bash
ros2 launch orca_bringup sim_launch.py
~~~

Execute a mission in a 2nd terminal:
~~~
source src/orca4/setup.bash
ros2 run orca_bringup mission_runner.py
~~~

### Using MAVProxy

It is possible to launch Gazebo and ArduSub and control the sub using MAVProxy.

Launch a minimal system:
~~~
ros2 launch orca_bringup sim_launch.py base:=false mavros:=false nav:=false rviz:=false slam:=false
~~~

Launch MAVProxy in a 2nd terminal:
~~~
mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out udp:0.0.0.0:14550 --console
~~~

You can use MAVProxy to send commands directly to ArduSub:
~~~
arm throttle
rc 3 1450
rc 3 1500
mode alt_hold
disarm
~~~

RC channels:
* RC 3 -- vertical
* RC 4 -- yaw
* RC 5 -- forward
* RC 6 -- strafe

### MAVProxy + SLAM

This will bring up a minimal system with SLAM and RViz:
~~~
ros2 launch orca_bringup sim_launch.py base:=false mavros:=false nav:=false
~~~

You can use MAVProxy to drive the sub around the seafloor and build a map.

## Video pipeline

The simulation uses Gazebo camera sensors to generate uncompressed 800x600 images in an
ideal stereo configuration. The frame rate is throttled to 5Hz to reduce CPU load in ORB_SLAM2, but
it can easily go higher.

## Change the cameras
If you want to change the cameras, it's necessary to do camera calibration.
For details, please go through [here](git@github.com:HKUST-UROP-ROV-SIM/gazebo_camera_calibration.git).
Once calibration is completed, please copy the parameters to the cfg folder for storage. Then, replace the parameters of the ```sim_left.ini``` and ```sim_right.ini```. Also, you need to provide the baseline of the cameras in the sim_orca_params.yaml.

## 3D point cloud
After finishing scanning, you may want to save the 3D point cloud.
Please provide the file name, directory and run:
```bash
ros2 service call /orb_slam2_stereo_node/save_map orb_slam2_ros/srv/SaveMap name:\ \'\'\ 
``` 

## Load 3D point cloud during launch
In the sim_orca_params.yaml, set ```map_file``` to your desired map and set ```load_map``` to ```True```.