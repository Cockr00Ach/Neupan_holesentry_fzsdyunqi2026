# fzsd2026 · 哨兵机器人导航工作区（NeuPAN 3D 局部规划 + Gazebo 仿真 + 网页端）

RoboMaster 2026 哨兵机器人的 ROS 2 工作区：`rmul_2025` 场地的 Gazebo 仿真、Nav2 导航栈，
以及把 **NeuPAN（3D 盒模型）** 接进仿真闭环的完整流水线——含网页端（RViz 搬到浏览器）、
SLAM 栅格地图与实时参数调节。

```
上层决策 / 目标点  →  参考路径（栅格 A*：静态地图 + 滚动栅格，带净空约束）
                    →  NeuPAN 3D 盒模型局部规划（DUNE + NRMP，0.1 s 步长）
                    →  /<robot>/robot_base/chassis_cmd  →  Gazebo 麦轮底盘
                    →  网页端（127.0.0.1:8088）：地图/点云/路径/机器人/参数滑条/发目标
```

## 环境

* Ubuntu 22.04、ROS 2 Humble、Ignition Gazebo 6（`ign gazebo 6.18`）
* Python: `torch`、`cvxpy`、`numpy`、`scipy`、`PyYAML`、`flask`、`PIL`（irsim 仅离线演示需要）

## 编译

```bash
cd fzsd2026
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 6
source install/setup.bash
```

> 若某个 Python 包报 `error: option --editable not recognized`（新版 setuptools 与
> `--symlink-install` 不兼容），删掉该包的 `build/<pkg>`、`install/<pkg>` 后
> `colcon build --packages-select <pkg>`（不加 `--symlink-install`）即可。

## 跑起来（NeuPAN 全链路，推荐）

```bash
cd src/NeuPAN/example/fzsd2026

./run_sim.sh --web --slam --nav-only     # 无界面 Gazebo + 两台车 + NeuPAN 桥接 + 网页 + SLAM
# 浏览器打开 http://127.0.0.1:8088/ ，按 G 进目标模式点地图发目标，S 急停，右侧可实时调参
./stop_sim.sh                            # 收工
```

常用变体：

```bash
./run_sim.sh                          # 按 planner.yaml 的 waypoints 自动跑一圈
./run_sim.sh --rviz                   # 用 RViz 看（map 坐标系）
./run_sim.sh --gmap map/rmul_2025_slam.yaml   # 用扫好的静态地图做全局 A*
./run_sim.sh --nav-only               # 生成机器人时不带 1080p 相机（实时率 1.0，默认 0.7）
bash check_all.sh                     # 四个验收场景（启动路线/动态绕障/单目标/网页收发）
```

详细文档见 **`src/NeuPAN/example/fzsd2026/README.md`**（架构、参数表、SLAM 建图、调参实测、
修过的仿真侧问题、排错）。

## 其它脚本（仓库根目录）

| 脚本 | 用途 |
| --- | --- |
| `colcon.sh` | 带 `--symlink-install` 的编译 |
| `nav.sh` | 实车导航（`rm_sentry_reality_launch.py`，`world:=rmul_2025`） |
| `map.sh` | 用 map_saver 存地图 |
| `slam.sh` / `tf.sh` / `command.sh` | 建图、TF 树查看、遥控 |
| `cleanup_sim.sh` | 清掉残留的仿真进程（避免话题重复） |

## 目录

```
src/
├── NeuPAN/                     NeuPAN 上游 + example/fzsd2026（3D 改造、桥接、网页端、SLAM）
├── fzsd2026_gazebo_simulator/  Gazebo 仿真、rmoss 驱动/插件（含修的底盘坐标系与灯光插件）
├── fzsd2026_sentry_nav/        Nav2 导航栈（bringup、地图、定位…）
├── fzsd2026_robot_description/ 机器人 SDF/URDF、雷达与相机模型
├── fzsd_rm_interfaces/         自定义消息
└── fzsd_vision_serial_driver/  视觉串口驱动
Backup/ rmul_2025.pgm|yaml       场地地图备份
small_gicp/ SentryWeave/         小 GICP 定位库、RL 研究代码
```

## 注意

* 仓库里**没有**放录制的传感器数据与先验点云（`recorddata/`、`*.pcd`、`*.tar.xz`，共十几 GB），
  需要时从原工作区拷过来，或在本地重新录/建图。
* `install/`、`build/`、`log/` 都是本地编译产物，不纳入版本管理。
* 仿真侧的两个 C++ 修复（`MecanumDrive2` 速度环坐标系、`LightBarController` 启动崩溃）已在
  `src/fzsd2026_gazebo_simulator/rmoss_gazebo/rmoss_gz_plugins/` 里，重新 `colcon build` 生效。
