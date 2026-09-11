# 北云 C2 ROS1 GNSS 驱动

该包从北云 C2 的串口字节流中按 `\n` 切分语句，校验并解析 BESTPOSA 和 RMC，然后发布 `sensor_msgs/NavSatFix`。

默认串口为 `/dev/ttyUSB0`，可以通过 ROS 私有参数 `~port` 修改。端口映射完成后只需修改 launch 文件中的 `port`，不需要改代码。

## 运行

```bash
roslaunch beiyun_gnss beiyun_gnss.launch
```

主要参数：

- `~port`：串口设备，默认 `/dev/ttyUSB0`
- `~baudrate`：波特率，默认 `115200`
- `~navsatfix_topic`：NavSatFix 话题，默认 `/rtk/navsatfix`
- `~pair_tolerance_seconds`：BESTPOSA GPS 时间与 RMC UTC 时间的配对容差，默认 `0.2`

发布话题：

- `/rtk/navsatfix`：有效 BESTPOSA + 有效 RMC 配对后的 `sensor_msgs/NavSatFix`
- `/rtk/raw_rmc`：原始 RMC 字符串
- `/rtk/raw_gga`：原始 GGA 字符串
- `/rtk/raw_bestposa`：原始北云 `#BESTPOSA` 字符串

## C2 配置要求

GNSS 驱动使用 C2 的 COM3，建议配置为：

```text
INTERFACEMODE COM3 BYNAV BYNAV
LOG COM3 GPRMC ONTIME 1
LOG COM3 BESTPOSA ONTIME 1
```

只有当 RMC 状态为 `A`、BESTPOSA 时间有效、解状态为 `SOL_COMPUTED` 且经纬度和标准差有效时才会发布 NavSatFix。GGA 可以继续输出用于现场诊断，但不参与正式消息组装。

## 验证

```bash
rostopic echo /rtk/navsatfix
rostopic hz /rtk/navsatfix
rostopic echo /rtk/raw_rmc
```

正常情况下，`header.stamp` 由 RMC 的完整 UTC 日期时间转换得到。经纬度、高程和解状态来自 BESTPOSA；BESTPOSA 的经度、纬度和高程标准差会分别平方后填入 `position_covariance` 的 east/north/up 对角线。
