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