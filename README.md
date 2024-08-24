# bluerov2_heavy_sim
## How to setup simulator?
1. Build the Dockerfile
```bash
cd bluerov2_heavy_sim
git submodule update --init --recursive
cd orca4/docker
./build.sh
```
2. Run the docker container, please configure the run.sh according to your computer's graphics card.
```bash
cd bluerov2_heavy_sim
./run.sh
```
3. Build the colcon_ws
```bash
colcon build
```
4. Launch simulation
```bash
. install/setup.bash
source src/orca4/setup.bash
ros2 launch orca_bringup sim_launch.py
```
5. If you want to scan the seabed model, open a new terminal and run
```bash
ros2 run orca_bringup mission_runner.py
```
