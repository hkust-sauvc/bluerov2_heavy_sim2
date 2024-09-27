# bluerov2_heavy_sim
## How to setup simulator?
<del>1. Install Git LFS      
<del>If Git LFS isn't isntalled, [install here](https://docs.github.com/en/repositories/working-with-files/managing-large-files/installing-git-large-file-storage).
2. Clone the repo
```bash
git clone git@github.com:HKUST-UROP-ROV-SIM/bluerov2_heavy_sim.git
```
3. Build the Dockerfile
```bash
cd ~/bluerov2_heavy_sim
git submodule update --init --recursive
<del>git lfs fetch
<del>git lfs checkout
cd orca4/docker
./build.sh
```
4. Run the docker container, please configure the run.sh according to your computer's graphics card.
```bash
cd ~/bluerov2_heavy_sim
./run.sh
```
5. Build the colcon_ws
```bash
colcon build
```
6. Launch simulation
```bash
. install/setup.bash
source src/orca4/setup.bash
ros2 launch orca_bringup sim_launch.py
```
7. If you want to scan the seabed model, open a new terminal and run
```bash
ros2 run orca_bringup mission_runner.py
```
