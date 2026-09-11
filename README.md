# LIGO 北云 GNSS + Livox 激光雷达建图工作区

这是一个基于 ROS1 的多传感器建图工作区，当前主要用于将北云 GNSS、Livox LiDAR/IMU 和 LIGO 建图算法组合起来，形成带 UTC 时间戳的外部 GNSS ENU 里程计，并参与 LIGO 的全局定位与建图。

## 系统流程

```text
北云 GNSS 板卡
  ├─ 串口输出 GPRMC / GPGGA / BESTPOSA
  │
  ├─ beiyun_gnss
  │    └─ BESTPOSA + GPRMC -> /rtk/navsatfix
  │
  └─ Livox 时间同步串口 + PPS
       └─ livox_ros_driver -> UTC 时间基准的 /livox/lidar 和 /livox/imu

/rtk/navsatfix
  └─ gnss_to_enu
       └─ LLA -> ECEF -> 局部 ENU -> /rtk/gps_odom

/livox/lidar + /livox/imu + /rtk/gps_odom
  └─ LIGO
       └─ LiDAR-Inertial 建图与 NMEA/ENU 外部里程计融合
```

当前推荐的正式消息链路是：

- `/rtk/navsatfix`：`sensor_msgs/NavSatFix`
- `/rtk/gps_odom`：`nav_msgs/Odometry`
- `/livox/lidar`：Livox 点云
- `/livox/imu`：Livox IMU

## 主要特性

- 北云 `#BESTPOSA` 提供经纬度、高程、解算状态和位置标准差。
- 北云 `GPRMC` 提供完整 UTC 日期和时间。
- 只有时间匹配的 BESTPOSA 与 GPRMC 才组成一条正式 `NavSatFix`。
- `gnss_to_enu` 透传 GNSS UTC 时间戳，不使用 `ros::Time::now()`。
- NavSatFix 经纬高被转换为局部 ENU 坐标，并发布为 `/rtk/gps_odom`。
- GNSS 三个方向的方差会在 `gnss_to_enu` 中分别检查。
- LIGO 的 NMEA 输入使用标准 `Odometry.pose.covariance` 对角线位置 `[0]`、`[7]`、`[14]` 做协方差门控。
- 当前 LIGO 因子噪声仍由 `pos_noise` 和 `nmea_weight` 等参数控制，不会根据每条 Odometry 的协方差动态改变。

## 环境要求

当前工作区按以下环境组织：

- Ubuntu 20.04
- ROS Noetic
- catkin
- C++11/C++17 编译器
- Eigen
- PCL
- GTSAM
- Livox SDK / `livox_ros_driver`

## 编译

在工作区根目录执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/pc/LIGO_ws
catkin_make
source devel/setup.bash
```

如果使用其他工作区路径，请将 `/home/pc/LIGO_ws` 替换为实际路径。

## 启动顺序

### 1. 启动北云 GNSS 驱动

当前 launch 默认读取：

```text
/dev/ttyACM1
```

启动：

```bash
roslaunch beiyun_gnss beiyun_gnss.launch
```

### 2. 启动 Livox LiDAR/IMU 驱动

```bash
roslaunch livox_ros_driver livox_lidar_msg.launch
```

Livox 时间同步配置位于：

```text
src/sensors/livox_ros_driver/livox_ros_driver/config/livox_lidar_config.json
```

其中 `timesync_config.enable_timesync` 应保持为 `true`，`device_name` 应指向给 Livox 驱动提供 GPRMC 的串口设备。

### 3. 启动 GNSS 到 ENU 转换节点

```bash
roslaunch gnss_to_enu gnss_to_enu_slam.launch
```

该节点默认订阅 `/rtk/navsatfix`，发布 `/rtk/gps_odom`。

### 4. 启动 LIGO 建图

```bash
roslaunch ligo mapping_avia_beiyun.launch
```

该 launch 加载：

```text
src/LIGO/config/avia_beiyun.yaml
```

并启用 NMEA/外部 ENU 里程计输入。

## 串口与时间同步注意事项

北云板卡可以通过多个串口输出不同数据。`beiyun_gnss` 需要独占它配置的串口，例如 `/dev/ttyACM1`，读取 BESTPOSA 和 RMC 并发布 NavSatFix。

Livox 驱动的 `timesync_config.device_name` 应使用另一个可输出 GPRMC 的串口端点。不要让两个独立程序同时打开同一个 `/dev/tty*` 设备，否则可能发生串口读取冲突。推荐使用北云板卡的两个串口输出，或者通过 udev 为两个物理串口建立稳定的设备名。

硬件同步链路应满足：

1. 北云输出 GPRMC 给 Livox 时间同步接口；
2. 北云 PPS 信号连接到 Livox 的 PPS 输入；
3. LiDAR、IMU 和 GNSS 消息使用同一 UTC 时间基准；
4. `/rtk/gps_odom.header.stamp` 与 LiDAR/IMU 时间戳位于同一时间轴。

## 运行检查

```bash
rostopic list
rostopic hz /rtk/navsatfix
rostopic hz /rtk/gps_odom
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic echo -n 1 /rtk/navsatfix
rostopic echo -n 1 /rtk/gps_odom
```

重点检查：

- `/rtk/navsatfix` 是否有输出；
- `header.stamp` 是否为 GNSS UTC 时间；
- NavSatFix 的经纬高是否有效；
- `position_covariance` 是否为合理的方差值；
- `/rtk/gps_odom` 的 `pose.covariance[0]`、`[7]`、`[14]` 是否正确；
- LiDAR、IMU 和 GNSS 时间戳是否处于同一时间基准；
- LIGO 是否订阅 `/rtk/gps_odom`。

## 目录说明

```text
docs/                         北云 C2 数据通信协议
src/sensors/beiyun_gnss/      北云串口驱动和 BESTPOSA/RMC 解析
src/gnss_to_enu/              NavSatFix 到局部 ENU Odometry 的转换
src/sensors/livox_ros_driver/ Livox LiDAR/IMU 驱动及时间同步
src/LIGO/                     LIGO 建图算法
```

想查看两个定制包的详细实现，请阅读：

- [gnss_to_enu 子包说明](src/gnss_to_enu/README.md)
- [beiyun_gnss 子包说明](src/sensors/beiyun_gnss/README.md)

北云 BESTPOSA 字段定义参见：

- [UG016 数据通信接口协议](<docs/UG016_数据通信接口协议_北云科技 .pdf>)

## 当前实现边界

- GPGGA 当前保留为原始诊断话题，不参与正式 NavSatFix 的位置、状态或协方差组装。
- BESTPOSA 的经纬度和高程标准差会平方后写入 NavSatFix 协方差。
- `gnss_to_enu` 当前会将 Z 方向方差额外放大 10 倍，以降低高度观测对后端的影响。
- LIGO 当前使用协方差阈值进行硬过滤，但 NMEA 因子噪声仍是静态配置，不是逐帧动态加权。
- `ppp_std_thres` 的历史命名包含 `std`，但当前代码实际比较的是方差，单位应按 `m^2` 理解
  (且当前配置为了1000，意味着方差大于1000的ENU里程即数据才会被LIGO内部过滤，所以说经过gnss_to_enu过滤后的ENU里程即数据完全不会被LIGO内部再过滤)。
