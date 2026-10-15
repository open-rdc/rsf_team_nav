#### Install

step0
```
mkdir -p ~/colcon_ws
```

step1
```
cd ~/colcon_ws
git clone https://github.com/kyo0221/rsf_team_nav.git src
vcs import src < src/navigation.repos
```

utility
```
vcs import src < src/utility.repos
```

deploy
```
vcs import src < src/deployment.repos
```

#### ypspur setup
```
cd src/yp-spur
mkdir build
cd build
cmake ..
make
sudo make install
cd ~/colcon_ws
```

rosdep
```
cd ~/colcon_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys catkin
```

step2
```
colcon build --symlink-install
```

#### DDS
Cyclone DDS を使う。`~/.bashrc` に追記し、切り替えた後は `ros2 daemon stop` を実行する。
```
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/colcon_ws/src/rsf_bringup/config/cyclonedds.xml
```
別 PC の RViz などから大きなトピックを受け取る場合は、両方の PC で受信バッファを広げる。
```
sudo sysctl -w net.core.rmem_max=2147483647 net.core.rmem_default=8388608
```
