#ifndef BEIYUN_GNSS_NMEA_PARSER_H_
#define BEIYUN_GNSS_NMEA_PARSER_H_

#include <string>

namespace beiyun_gnss {

// UTC 时间拆成日期和时分秒，RMC 提供完整字段，GGA 只提供时分秒。
struct UtcTime {
  int year;
  int month;
  int day;
  int hour;
  int minute;
  double second;

  UtcTime();
};

// RMC 主要用于提供完整 UTC 时间；其中的坐标只作为可选信息保留。
struct RmcData {
  UtcTime utc;
  bool valid;
  bool has_position;
  double latitude;
  double longitude;

  RmcData();
};

// GGA 主要用于提供经纬度、高程和定位质量。
struct GgaData {
  UtcTime utc;
  bool valid;
  double latitude;
  double longitude;
  double altitude;
  int fix_quality;
  int satellites;
  double hdop;

  GgaData();
};

// 解析带有效 NMEA 校验和的 GPRMC/GNRMC 和 GPGGA/GNGGA 语句。
// 返回 false 表示语句格式、校验和或定位状态不满足发布条件。
bool ParseRmc(const std::string &sentence, RmcData *data);
bool ParseGga(const std::string &sentence, GgaData *data);

// GGA 没有日期，因此配对时比较时分秒；同时处理跨午夜的边界情况。
bool UtcClose(const UtcTime &rmc_time, const UtcTime &gga_time,
              double tolerance_seconds);

}  // namespace beiyun_gnss

#endif  // BEIYUN_GNSS_NMEA_PARSER_H_
