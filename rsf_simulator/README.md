# rsf_simulator

Gazebo (Ignition Fortress / Gazebo Harmonic) 上に RSF ロボットとワールドを起動する。

## 起動

ナビゲーション用のロボットTFとGazeboをまとめて起動する場合:

```bash
ros2 launch rsf_bringup rsf_bringup.launch.py
```

次に別端末からNav2を起動し、waypoint実行サービスを呼び出す:

```bash
ros2 launch rsf_navigation_executor rsf_navigation.launch.py
ros2 service call /waypoint_navigator/start std_srvs/srv/Trigger '{}'
```

ヘッドレスで起動する場合は `gz_args:=-s -r -v 2` をbringupに渡す。
この手順では `rsf_simulator.launch.py` を別途起動しない。

Gazeboだけを個別に起動する場合:

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py
```

## ワールドの選択

| `world` |
|---|
| `tsudanuma2-3`（デフォルト）|
| `tsudanuma` |

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py world:=tsudanuma
```

`worlds/` に `<名前>.sdf` を置き、launch の `choices` に名前を追加すれば選択肢を増やせる。


## topic

`ros_gz_bridge` が以下を ROS 側に橋渡しする。

| ROS トピック | 向き | 内容 |
|---|---|---|
| `/clock` | output | シミュレーション時刻 |
| `/rsf/hokuyo_cloud2` | output | 3D LiDAR の点群|
| `/rsf/imu` | output | IMU |
| `/rsf/nav_sat_fix` | output | GNSS |
| `/rsf/rsf_odom` | output | オドメトリ |
| `/cmd_vel` | input | 速度指令 |

トピック名とフレーム名は実機ドライバ `hokuyo_rsf` の設定に揃えてあり、
実機とシミュレータで下流ノードの設定を変えずに済むようにしてある。
