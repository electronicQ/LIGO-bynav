//使用标准的 POSIX TTY API，
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <limits>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/String.h>

#include "beiyun_gnss/covariance_policy.h"
#include "beiyun_gnss/nmea_parser.h"

namespace {

// ROS 参数使用整数波特率，termios 使用系统常量，二者在这里做映射。
speed_t BaudToTermios(int baudrate) {
  switch (baudrate) {
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    default: return 0;
  }
}

// 将 RMC 的完整 UTC 日期时间转换为 ROS 时间戳。
// timegm 明确按 UTC 解释 tm，避免受电脑本地时区影响。
ros::Time ToRosTime(const beiyun_gnss::UtcTime &utc) {
  std::tm calendar = {};
  calendar.tm_year = utc.year - 1900;
  calendar.tm_mon = utc.month - 1;
  calendar.tm_mday = utc.day;
  calendar.tm_hour = utc.hour;
  calendar.tm_min = utc.minute;
  calendar.tm_sec = static_cast<int>(std::floor(utc.second));
  const std::time_t epoch_seconds = timegm(&calendar);
  if (epoch_seconds < 0) return ros::Time(0);
  const double fractional = utc.second - std::floor(utc.second);
  uint32_t nanoseconds = static_cast<uint32_t>(std::llround(fractional * 1e9));
  std::time_t adjusted_epoch = epoch_seconds;
  if (nanoseconds >= 1000000000U) {
    ++adjusted_epoch;
    nanoseconds -= 1000000000U;
  }
  return ros::Time(static_cast<uint32_t>(adjusted_epoch), nanoseconds);
}

// 只用于快速路由语句类型，真正的数据合法性由 nmea_parser 校验。
bool IsPrefixed(const std::string &line, const char *prefix) {
  return line.compare(0, std::strlen(prefix), prefix) == 0;
}

// POSIX 串口封装：负责打开设备、配置 8N1/波特率/超时、读取字节和关闭设备。
// 设备文件由 ROS 参数提供，因此 /dev/ttyUSB0 可以替换为后续的 udev 映射名。
class SerialPort {
 public:
  SerialPort() : fd_(-1) {}
  ~SerialPort() { Close(); }

  // 以原始模式打开串口。C2 的 NMEA 输出是普通字节流，不需要行编辑或软件流控。
  bool Open(const std::string &device, int baudrate, double timeout_seconds,
            std::string *error) {
    Close();
    const speed_t speed = BaudToTermios(baudrate);
    if (speed == 0) {
      if (error) *error = "unsupported baudrate";
      return false;
    }

    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd_ < 0) {
      if (error) *error = std::strerror(errno);
      return false;
    }

    struct termios settings;
    if (tcgetattr(fd_, &settings) != 0) {
      if (error) *error = std::strerror(errno);
      Close();
      return false;
    }
    cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~CRTSCTS;
    cfsetispeed(&settings, speed);
    cfsetospeed(&settings, speed);
    const int tenths = static_cast<int>(std::ceil(timeout_seconds * 10.0));
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = static_cast<cc_t>(std::max(1, std::min(255, tenths)));
    if (tcsetattr(fd_, TCSANOW, &settings) != 0) {
      if (error) *error = std::strerror(errno);
      Close();
      return false;
    }
    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  // 关闭文件描述符；下次 Run 循环会重新打开设备。
  void Close() {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

  // 一次读取可能得到半条、整条或多条语句，调用方必须自行进行分帧。
  ssize_t Read(char *buffer, size_t size, int *error_number) {
    const ssize_t count = read(fd_, buffer, size);
    if (count < 0 && error_number) *error_number = errno;
    return count;
  }

  bool IsOpen() const { return fd_ >= 0; }

 private:
  int fd_;
};

// 北云 GNSS ROS 节点：串口读取、语句缓存、GGA/RMC 配对和 ROS 发布均在此完成。
class BeiyunGnssNode {
 public:
  BeiyunGnssNode()
      : nh_(),
        private_nh_("~"),
        port_("/dev/ttyUSB0"),
        baudrate_(115200),
        serial_timeout_(0.1),
        frame_id_("gps"),
        navsatfix_topic_("/rtk/navsatfix"),
        pair_tolerance_seconds_(0.2),
        covariance_config_(),
        has_rmc_(false),
        has_gga_(false) {
    // 私有参数允许在端口映射前后使用同一个程序，不需要重新编译。
    private_nh_.param("port", port_, port_);
    private_nh_.param("baudrate", baudrate_, baudrate_);
    private_nh_.param("serial_timeout", serial_timeout_, serial_timeout_);
    private_nh_.param("frame_id", frame_id_, frame_id_);
    private_nh_.param("navsatfix_topic", navsatfix_topic_, navsatfix_topic_);
    private_nh_.param("pair_tolerance_seconds", pair_tolerance_seconds_,
              pair_tolerance_seconds_);
    private_nh_.param("single_point_covariance_m2",
              covariance_config_.single_point_covariance_m2,
              covariance_config_.single_point_covariance_m2);
    private_nh_.param("dgps_covariance_m2", covariance_config_.dgps_covariance_m2,
              covariance_config_.dgps_covariance_m2);
    private_nh_.param("rtk_fixed_covariance_m2",
              covariance_config_.rtk_fixed_covariance_m2,
              covariance_config_.rtk_fixed_covariance_m2);
    private_nh_.param("rtk_float_covariance_m2",
              covariance_config_.rtk_float_covariance_m2,
              covariance_config_.rtk_float_covariance_m2);

    // NavSatFix 是融合后的正式输出；三个 raw 话题用于现场排查串口内容。
    navsatfix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(navsatfix_topic_, 10);
    raw_rmc_pub_ = nh_.advertise<std_msgs::String>("/rtk/raw_rmc", 10);
    raw_gga_pub_ = nh_.advertise<std_msgs::String>("/rtk/raw_gga", 10);
    raw_bestposa_pub_ = nh_.advertise<std_msgs::String>("/rtk/raw_bestposa", 10);
  }

  void Run() {
    // 主循环负责断线重连和读取。串口本身使用 VTIME，ROS 节点仍保持 100 Hz 响应。
    ros::Rate loop_rate(100.0);
    while (ros::ok()) {
      if (!serial_.IsOpen()) {
        std::string error;
        if (serial_.Open(port_, baudrate_, serial_timeout_, &error)) {
          ROS_INFO("Opened Beiyun GNSS serial %s at %d baud", port_.c_str(),
                   baudrate_);
        } else {
          ROS_WARN_THROTTLE(2.0, "Cannot open GNSS serial %s: %s", port_.c_str(),
                            error.c_str());
          ros::spinOnce();
          loop_rate.sleep();
          continue;
        }
      }

      // read() 返回的是任意长度字节块，不能假定一次对应一条 NMEA 语句。
      char bytes[512];
      int error_number = 0;
      const ssize_t count = serial_.Read(bytes, sizeof(bytes), &error_number);
      if (count > 0) {
        receive_buffer_.append(bytes, static_cast<size_t>(count));
        ProcessBuffer();
      } else if (count < 0 && error_number != EINTR && error_number != EAGAIN &&
                 error_number != EWOULDBLOCK) {
        ROS_WARN("GNSS serial read failed: %s; reconnecting", std::strerror(error_number));
        serial_.Close();
        receive_buffer_.clear();
        has_rmc_ = false;
        has_gga_ = false;
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

 private:
  // 从累计字节流中取出以 '\n' 结尾的完整行，并去掉 C2 使用的 '\r'。
  void ProcessBuffer() {
    while (true) {
      const std::string::size_type newline = receive_buffer_.find('\n');
      if (newline == std::string::npos) {
        if (receive_buffer_.size() > 4096) {
          ROS_WARN_THROTTLE(2.0, "Discarding overlong GNSS serial line");
          receive_buffer_.clear();
        }
        return;
      }
      std::string line = receive_buffer_.substr(0, newline);
      receive_buffer_.erase(0, newline + 1);
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (!line.empty()) HandleLine(line);
    }
  }

  // 先按语句类型发布原始文本，再把有效 GGA/RMC 放入配对缓存。
  // BESTPOSA 没有直接参与 NavSatFix 组装，仅保留供诊断使用。
  void HandleLine(const std::string &line) {
    std_msgs::String raw;
    raw.data = line;
    if (IsPrefixed(line, "$GPRMC") || IsPrefixed(line, "$GNRMC")) {
      raw_rmc_pub_.publish(raw);
      beiyun_gnss::RmcData parsed;
      if (beiyun_gnss::ParseRmc(line, &parsed)) {
        rmc_ = parsed;
        has_rmc_ = true;
        TryPublishFix();
      }
    } else if (IsPrefixed(line, "$GPGGA") || IsPrefixed(line, "$GNGGA")) {
      raw_gga_pub_.publish(raw);
      beiyun_gnss::GgaData parsed;
      if (beiyun_gnss::ParseGga(line, &parsed)) {
        gga_ = parsed;
        has_gga_ = true;
        TryPublishFix();
      }
    } else if (IsPrefixed(line, "#BESTPOSA")) {
      raw_bestposa_pub_.publish(raw);
    }
  }

  // 只有 RMC 和 GGA 的 UTC 时刻匹配时才发布，时间戳使用 RMC 的完整日期时间。
  // last_published_stamp_ 防止同一组缓存因两条语句到达顺序而重复发布。
  void TryPublishFix() {
    if (!has_rmc_ || !has_gga_ ||
        !beiyun_gnss::UtcClose(rmc_.utc, gga_.utc, pair_tolerance_seconds_)) {
      return;
    }
    const ros::Time stamp = ToRosTime(rmc_.utc);
    if (!last_published_stamp_.isZero() && stamp == last_published_stamp_) return;

    sensor_msgs::NavSatFix message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.status.status = StatusForFixQuality(gga_.fix_quality);
    message.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    message.latitude = gga_.latitude;
    message.longitude = gga_.longitude;
    message.altitude = gga_.altitude;
    // 根据 GGA quality 选择对应方差，并填入 east/north/up 三个对角元素。
    message.position_covariance.fill(0.0);
    const double covariance = beiyun_gnss::CovarianceForFixQuality(
        gga_.fix_quality, covariance_config_);
    if (covariance > 0.0 && std::isfinite(covariance)) {
      message.position_covariance[0] = covariance;
      message.position_covariance[4] = covariance;
      message.position_covariance[8] = covariance;
      message.position_covariance_type =
          sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    } else {
      message.position_covariance_type =
          sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }
    navsatfix_pub_.publish(message);
    last_published_stamp_ = stamp;
  }

  // 将 GGA fix quality 映射为 sensor_msgs/NavSatStatus 状态码。
  static int8_t StatusForFixQuality(int quality) {
    switch (quality) {
      case 2: return sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
      case 4:
      case 5: return sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
      case 1: return sensor_msgs::NavSatStatus::STATUS_FIX;
      default: return sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    }
  }

  // ROS、串口和参数
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  SerialPort serial_;
  std::string port_;
  int baudrate_;
  double serial_timeout_;
  std::string frame_id_;
  std::string navsatfix_topic_;
  double pair_tolerance_seconds_;
  beiyun_gnss::CovarianceConfig covariance_config_;
  // 串口字节流和最近一条有效 GGA/RMC
  std::string receive_buffer_;
  beiyun_gnss::RmcData rmc_;
  beiyun_gnss::GgaData gga_;
  bool has_rmc_;
  bool has_gga_;
  ros::Time last_published_stamp_;
  // ROS 发布器
  ros::Publisher navsatfix_pub_;
  ros::Publisher raw_rmc_pub_;
  ros::Publisher raw_gga_pub_;
  ros::Publisher raw_bestposa_pub_;
};

}  // namespace

int main(int argc, char **argv) {
  try {
    ros::init(argc, argv, "beiyun_gnss_node");
    BeiyunGnssNode node;
    node.Run();
    return 0;
  } catch (const std::exception &e) {
    fprintf(stderr, "beiyun_gnss_node terminated by exception: %s\n", e.what());
  } catch (...) {
    fprintf(stderr, "beiyun_gnss_node terminated by unknown exception\n");
  }
  return 1;
}
