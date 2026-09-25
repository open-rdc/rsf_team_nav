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

デプロイ時
```
vcs import src < src/deployment.repos
```
#### ypspur setup
```
cd src/yps-pur
mkdir build
cd build
cmake ..
make
sudo make install
cd ~/colcon_ws
```

step2
```
colcon build --symlink-install
```
