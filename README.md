# SmartCar · ROS 2 基础自主避障

这是一辆在 Gazebo Classic 11 中运行的差速小车。它根据二维激光雷达数据低速直行、原地转向和安全停车。默认世界有三个测试箱体，四面短墙围出约 6.3 m × 6.3 m 的测试区域。

## 工程结构

```text
smart_car/
├── .gitignore
├── README.md
├── docker/Dockerfile
├── smart_car_description/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── urdf/smart_car.urdf.xacro
├── smart_car_gazebo/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/simulation.launch.py
│   └── worlds/patrol.world
└── smart_car_control/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/smart_car_control
    ├── smart_car_control/
    │   ├── __init__.py
    │   ├── decision.py
    │   └── obstacle_avoidance.py
    └── test/test_decision.py
```

| 数据流 | 作用 |
| --- | --- |
| Gazebo Classic 雷达插件 → `/scan` → `/obstacle_avoidance` | 发布 `LaserScan`，`laser_link` 坐标系，10 Hz，0.10–8.00 m，721 束 |
| `/obstacle_avoidance` → `/cmd_vel` → Gazebo 差速插件 | 发布一个 `Twist` 指令流 |
| Gazebo 差速插件 → `/odom` 和 `/tf` | 提供轮速积分里程计和 `odom → base_link` |
| Gazebo 轮关节插件 → `/joint_states` → `/robot_state_publisher` | 发布轮子 TF 和固定的 `base_link → laser_link` |
| Gazebo → `/clock` | 提供仿真时间。停车看门狗使用独立的单调时钟 |

默认参数下，前方安全时进入 `CRUISE`，速度为 0.22 m/s。前方扇区小于 0.60 m 时进入 `AVOID`，线速度归零，选择余量较大的一侧以 0.65 rad/s 转向。选定方向保留到前方超过恢复阈值（0.82 m）或当前转向侧空间不足。转向侧最小余量为 0.30 m。扫描超时 0.50 s、扫描无效或两个转向侧都受阻时进入 `STOP`，持续发布零速度。有效扫描恢复后重新计算状态。

## Ubuntu 22.04 构建和运行

需要 ROS 2 Humble、Gazebo Classic 11、`gazebo_ros`、`gazebo_plugins`、`robot_state_publisher`、`xacro`、Python 3 和 colcon。已有这些依赖的 Ubuntu 主机可以直接构建。缺少依赖且软件源可用时，可安装：

```bash
sudo apt update
sudo apt install gazebo python3-colcon-common-extensions python3-pytest \
  ros-humble-gazebo-ros-pkgs ros-humble-robot-state-publisher ros-humble-xacro
```

将仓库放到 `~/smartcar_ws/src/smart_car`。GitHub 可访问时可以用：

```bash
mkdir -p ~/smartcar_ws/src
cd ~/smartcar_ws/src
git clone --branch main https://github.com/Han0920-op/smart_car.git
```

在工作空间中构建并一键启动：

```bash
source /opt/ros/humble/setup.bash
cd ~/smartcar_ws
colcon build --packages-select smart_car_description smart_car_control smart_car_gazebo
source install/setup.bash
ros2 launch smart_car_gazebo simulation.launch.py
```

有桌面显示时可加 `gui:=true`。若 OpenGL 初始化失败，使用 `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch smart_car_gazebo simulation.launch.py gui:=true`。默认以无界面方式运行。使用 Docker 时先从 `docker/` 构建镜像，再在容器的工作空间构建源码。Dockerfile 中编译的 Classic ROS 插件位于 `/opt/smartcar_classic`，构建前需额外执行 `source /opt/smartcar_classic/setup.bash`。

## 验证

在每个新终端执行 `source ~/smartcar_ws/install/setup.bash`。

### 自动模式

```bash
ros2 node list
ros2 topic list
ros2 topic info /cmd_vel -v
ros2 topic info /scan -v
ros2 topic echo /scan --once --full-length
ros2 run tf2_ros tf2_echo base_link laser_link
```

确认 `/cmd_vel` 只有一个发布者 `/obstacle_avoidance`。查看启动终端，在接近箱体或墙时观察 `CRUISE → AVOID → CRUISE`。日志用 `front/left/right` 报告三个扇区内的最近距离。另开终端可用 `gz model -w patrol -m smart_car -p` 查看仿真真值位姿，并用 `ros2 topic echo /odom --once` 查看轮式里程计。

在自动模式中验证看门狗：

```bash
ros2 service call /pause_physics std_srvs/srv/Empty '{}'
# 等日志出现 state=STOP reason=scan_timeout
ros2 topic echo /cmd_vel --once
ros2 service call /unpause_physics std_srvs/srv/Empty '{}'
```

暂停会同时停止物理仿真和雷达更新。此时观察的是看门狗是否继续发布零速度，以及恢复扫描后能否重新决策。

### 手动运动和单元测试

先用 Ctrl+C 结束自动仿真，再启动手动模式。只在此模式用 CLI 发送速度指令。

```bash
ros2 launch smart_car_gazebo simulation.launch.py avoidance:=false
```

在另一终端依次发送前进、停止、转弯、停止。比较每步前后的 Gazebo 位姿与里程计：

```bash
ros2 topic pub --times 20 --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
ros2 topic pub --times 20 --rate 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.4}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
gz model -w patrol -m smart_car -p
ros2 topic echo /odom --once
```

```bash
cd ~/smartcar_ws
colcon test --packages-select smart_car_control --return-code-on-test-failure
colcon test-result --verbose
```

### 云环境实测记录

以下数据来自 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Classic 11.10.2 的无界面运行。

| 检查 | 结果 |
| --- | --- |
| 构建与依赖 | 三个包构建成功，`rosdep check` 通过，8 项单元测试全部通过 |
| 雷达与 TF | `/scan` 为 721 束、10 Hz，`laser_link` 可按扫描时间戳查询。左、右障碍物的代表射线测得 1.447 m、1.630 m。前方箱体后移 1 m 后，正前方从 1.723 m 变为 2.725 m |
| 手动运动 | Gazebo 位姿显示直行 0.499 m，原地转过 1.203 rad。停车稳定后 1.5 秒内漂移约 0.05 mm |
| 自主连续运行 | 180.01 秒墙钟时间，对应 179.32 秒仿真时间。收到 1794 帧扫描和 8970 帧里程计，观察到 14 次转向后恢复前进。60 个 Gazebo 位姿采样估算行程约 34.77 m |
| 雷达失联 | 在物理仿真持续运行时断开控制器的扫描输入。最后一帧后 0.532 秒发出零速度，里程计线速度由 0.160 m/s 降至 0.0002 m/s。恢复输入后小车重新前进。暂停物理仿真时也确认看门狗会发出零速度 |
| 图形显示 | Gazebo GUI 和 RViz2 未在云环境中验证 |

## 三分钟会议演示

1. **0:00–0:45** 一键启动，介绍激光雷达、三个箱体和数据流。用 `ros2 topic info /scan -v` 和 `ros2 topic info /cmd_vel -v` 显示发布者。
2. **0:45–1:45** 观察前方距离下降触发 `AVOID`。展示线速度归零、角速度带符号，以及前方重新清空后恢复 `CRUISE`。
3. **1:45–2:30** 暂停仿真，等待 `STOP reason=scan_timeout`。查看 `/cmd_vel` 的零速度，再恢复仿真。
4. **2:30–3:00** 查看 `base_link → laser_link` TF 和 `/odom`，用 Ctrl+C 结束演示。
