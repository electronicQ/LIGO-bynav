# 北云 C2 ROS1 GNSS 驱动

该包从北云 C2 的串口字节流中按 `\n` 切分语句，校验并解析 `GPRMC/GNRMC` 和 `GPGGA/GNGGA`，然后发布 `sensor_msgs/NavSatFix`。

默认串口为 `/dev/ttyUSB0`，可以通过 ROS 私有参数 `~port` 修改。端口映射完成后只需修改 launch 文件中的 `port`，不需要改代码。

## 运行

```bash
roslaunch beiyun_gnss beiyun_gnss.launch
```

主要参数：

- `~port`：串口设备，默认 `/dev/ttyUSB0`
- `~baudrate`：波特率，默认 `115200`
- `~navsatfix_topic`：NavSatFix 话题，默认 `/rtk/navsatfix`
- `~pair_tolerance_seconds`：GGA/RMC UTC 时刻配对容差，默认 `0.2`
- `~single_point_covariance_m2`：单点定位位置方差，默认 `25.0`
- `~dgps_covariance_m2`：伪距差分位置方差，默认 `1.0`
- `~rtk_fixed_covariance_m2`：RTK 固定解位置方差，默认 `0.0004`
- `~rtk_float_covariance_m2`：RTK 浮动解位置方差，默认 `0.04`

发布话题：

- `/rtk/navsatfix`：有效 GPRMC + 有效 GPGGA 配对后的 `sensor_msgs/NavSatFix`
- `/rtk/raw_rmc`：原始 RMC 字符串
- `/rtk/raw_gga`：原始 GGA 字符串
- `/rtk/raw_bestposa`：原始北云 `#BESTPOSA` 字符串，仅作诊断，不参与 NavSatFix 组装

## C2 配置要求

GNSS 驱动使用 C2 的 COM3，建议配置为：

```text
INTERFACEMODE COM3 BYNAV BYNAV
LOG COM3 GPGGA ONTIME 1
LOG COM3 GPRMC ONTIME 1
```

只有当 RMC 的状态为 `A`、日期时间完整，且 GGA 的定位质量大于 0、经纬度有效时才会发布 NavSatFix。用户当前的 `V` 状态和空 GGA 是无效数据，驱动会等待 C2 获得定位。

## 验证

```bash
rostopic echo /rtk/navsatfix
rostopic hz /rtk/navsatfix
rostopic echo /rtk/raw_rmc
```

正常情况下，`header.stamp` 是由 RMC 的完整 UTC 日期和时间转换得到的 ROS 时间，GGA 只用于经纬高和定位质量。根据 GGA quality 选中的方差会填入 `position_covariance` 的 east/north/up 对角线，并将 `position_covariance_type` 设置为 `DIAGONAL_KNOWN`。这些方差是可配置的工程初始值，应根据 C2 厂商精度指标或实测数据校准。
