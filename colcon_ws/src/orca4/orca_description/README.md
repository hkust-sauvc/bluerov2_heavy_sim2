# Orca4 SDF Files

Orca4 uses the worlds and models from [bluerov2_ignition](https://github.com/clydemcqueen/bluerov2_ignition).

The [bluerov2](https://github.com/clydemcqueen/bluerov2_ignition/tree/main/models/bluerov2_heavy) model
was copied and modified to create the [orca4](models/orca4) model.

The [underwater](https://github.com/clydemcqueen/bluerov2_ignition/tree/main/worlds/underwater) world
was copied and modified to create the [sand](models/orca4) world.

The [generate_model.py](scripts/generate_model.py) script takes the [orca/model.sdf.in](models/orca4/model.sdf.in)
file and replaces strings of the form `@foo` with calculated values to generate
[orca/model.sdf](models/orca4/model.sdf).
The script must be run manually when the model is changed.

# UROP
Orca4 is equipped with 2 side cameras like the original Orca4 model.  
The name of the simulated vehicle is kept remain unchanged due to unresolvable name conflicts.

## DVL 
The ROV is now equipped with a DVL (nortek_dvl500_300). At this stage, only this DVL model doesn't cause any bug related to publishing sensor data. 
### How to add a DVL on the ROV?
To add a DVL, I copy the base link, ```<allow_auto_disable>1</allow_auto_disable>```, and DVLBridge plugin from the DVL model.sdf to the ROV model file.
If you want to add other DVL on the ROV, just need to copy similar parts of the DVL model to the ROV. 

Parts to be copied:
```xml
<link name="dvl500_300">
...
</link>
<allow_auto_disable>1</allow_auto_disable>
<plugin
    filename="DVLBridge"
    name="dave_ros_gz_plugins::DVLBridge">
...
</plugin>
```

In the world file, put this
```xml
  	<plugin
			filename="gz-sim-dvl-system"
			name="gz::sim::systems::DopplerVelocityLogSystem">
		</plugin>
```

