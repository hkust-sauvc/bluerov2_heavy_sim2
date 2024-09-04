# bluerov2_heavy_sim
## How to setup simulator?
1. Install Git LFS
If Git LFS isn't isntalled, go to [https://docs.github.com/en/repositories/working-with-files/managing-large-files/installing-git-large-file-storage](https://docs.github.com/en/repositories/working-with-files/managing-large-files/installing-git-large-file-storage) and install it.
2. Build the Dockerfile
```bash
cd ~/bluerov2_heavy_sim
git submodule update --init --recursive
git lfs fetch
git lfs checkout
cd orca4/docker
./build.sh
```
3. Run the docker container, please configure the run.sh according to your computer's graphics card.
```bash
cd ~/bluerov2_heavy_sim
./run.sh
```
4. Build the colcon_ws
```bash
colcon build
```
5. Launch simulation
```bash
. install/setup.bash
source src/orca4/setup.bash
ros2 launch orca_bringup sim_launch.py
```
6. If you want to scan the seabed model, open a new terminal and run
```bash
ros2 run orca_bringup mission_runner.py
```
