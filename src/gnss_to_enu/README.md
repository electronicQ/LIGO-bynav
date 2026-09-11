# gnss_to_enu

`gnss_to_enu` 将 `sensor_msgs/NavSatFix` 转换为局部 ENU 坐标系下的 `nav_msgs/Odometry`，供 LIGO 的 NMEA/外部里程计接口使用。

## 输入与输出

默认配置：

```text
输入：/rtk/navsatfix       sensor_msgs/NavSatFix
输出：/rtk/gps_odom        nav_msgs/Odometry
```

启动：

```bash
roslaunch gnss_to_enu gnss_to_enu_slam.launch
```

## 处理流程

### 1. 读取 NavSatFix

节点从 NavSatFix 读取：

- `latitude`：纬度，单位度；
- `longitude`：经度，单位度；
- `altitude`：椭球高，单位米；
- `position_covariance[0]`：东向位置方差；
- `position_covariance[4]`：北向位置方差；
- `position_covariance[8]`：天向位置方差；
- `header.stamp`：GNSS UTC 时间戳。

NavSatFix 的位置协方差单位为 `m^2`。

### 2. 三轴精度过滤

当前节点使用三个方向的方差进行过滤：

```python
if rtk_var_x > 2.0 or rtk_var_y > 2.0 or rtk_var_z > 2.0:
    return
```

其中：

```text
rtk_var_x = position_covariance[0]
rtk_var_y = position_covariance[4]
rtk_var_z = position_covariance[8]
```

任意一个方向的方差大于 `2.0 m^2`，当前 NavSatFix 都不会继续转换和发布。

这一步是前置硬过滤，不是后端动态降权。

### 3. 建立局部原点

默认 `use_auto_datum=true`，节点启动后接收到第一条通过精度过滤的 NavSatFix 时，将该帧的经纬高作为 ENU 原点。

原点只设置一次，后续位置都相对于该原点计算。

如果设置 `use_auto_datum=false`，则使用 launch 或参数提供的：

```text
datum_lat
datum_lon
datum_alt
```

### 4. LLA 转 ENU

坐标转换过程为：

```text
WGS84 LLA
  -> ECEF
  -> 局部 ENU
```

输出坐标含义：

```text
Odometry.pose.pose.position.x = East
Odometry.pose.pose.position.y = North
Odometry.pose.pose.position.z = Up
```

### 5. 时间和空间降采样

节点支持两个发布限制：

- `min_T_intervel`：两次发布之间的最小时间间隔，单位秒；
- `min_D_interval`：两次发布之间的最小水平距离，单位米。

当前 launch 使用：

```xml
<param name="min_T_intervel" value="1.0" />
<param name="min_D_interval" value="1.0" />
```

因此只有时间间隔和水平移动距离都满足条件时，才会发布新的 ENU 里程计。

### 6. 组装 Odometry

输出消息的主要字段为：

```text
header.stamp       = 输入 NavSatFix.header.stamp
header.frame_id    = "map"
child_frame_id     = "gps_link"
pose.position      = 局部 ENU 坐标
pose.orientation   = 单位四元数
```

时间戳直接透传 GNSS NavSatFix 的 UTC 时间，不使用 `rospy.Time.now()`。这样 `/rtk/gps_odom` 可以与已经通过 PPS/GPRMC 同步的 LiDAR 和 IMU 使用同一时间轴。

## Odometry 协方差映射

输出使用标准 `nav_msgs/Odometry.pose.covariance` 6x6 行主序布局：

```text
[0]  X-X 方差
[7]  Y-Y 方差
[14] Z-Z 方差
```

当前映射为：

```text
pose.covariance[0]  = NavSatFix.position_covariance[0]
pose.covariance[7]  = NavSatFix.position_covariance[4]
pose.covariance[14] = NavSatFix.position_covariance[8] * 10.0
```

其中 X/Y/Z 分别对应 East/North/Up。Z 方向额外乘以 `10.0` 是当前实现中的高度降权策略。

姿态没有 GNSS 观测，因此：

```text
pose.covariance[21] = 100.0
pose.covariance[28] = 100.0
pose.covariance[35] = 100.0
```

其余未使用的协方差项保持为 `0.0`。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `~sub_topic` | `/chcnav_fix_demo/fix` | 输入 NavSatFix 话题 |
| `~pub_topic` | `/rtk/gps_odom` | 输出 Odometry 话题 |
| `~use_auto_datum` | `true` | 是否使用第一条有效数据作为原点 |
| `~datum_lat` | `0.0` | 手动原点纬度 |
| `~datum_lon` | `0.0` | 手动原点经度 |
| `~datum_alt` | `0.0` | 手动原点高程 |
| `~min_T_intervel` | `0.0` | 最小发布时间间隔，秒 |
| `~min_D_interval` | `0.0` | 最小水平移动距离，米 |
| `~vehicle_mode` | `uav` | 车辆模式参数，保留用于配置兼容 |

当前 launch 文件已经将输入输出话题设置为：

```text
sub_topic = /rtk/navsatfix
pub_topic = /rtk/gps_odom
```

## 验证

```bash
rostopic echo -n 1 /rtk/navsatfix
rostopic echo -n 1 /rtk/gps_odom
rostopic hz /rtk/gps_odom
```

建议重点检查：

1. `/rtk/gps_odom.header.stamp` 是否与 `/rtk/navsatfix.header.stamp` 相同；
2. ENU 坐标是否满足 X 东、Y 北、Z 上；
3. `pose.covariance[0]`、`[7]`、`[14]` 是否为有效方差；
4. 任一输入方向方差超过 `2.0 m^2` 时是否停止发布。

## 与 LIGO 的接口

LIGO 配置中的 NMEA 话题为：

```yaml
nmea:
    nmea_enable: true
    posit_odo_topic: "/rtk/gps_odom"
```

LIGO 对外部 ENU 里程计使用的协方差对角项为：

```text
pose.covariance[0]
pose.covariance[7]
pose.covariance[14]
```

当前这些协方差主要用于 LIGO 的数据接收门控。LIGO 的 NMEA 因子噪声仍由其 `pos_noise` 和 `nmea_weight` 参数控制，尚未实现按每帧协方差动态构造因子噪声。
