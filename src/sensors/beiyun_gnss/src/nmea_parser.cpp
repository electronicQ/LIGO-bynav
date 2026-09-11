#include "beiyun_gnss/nmea_parser.h"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <vector>

namespace beiyun_gnss {
namespace {

// NMEA 校验和是 '$' 与 '*' 之间所有字符的逐字节 XOR，结果以两位十六进制表示。
bool IsHex(char value) {
  return (value >= '0' && value <= '9') ||
         (value >= 'a' && value <= 'f') ||
         (value >= 'A' && value <= 'F');
}

int HexValue(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  return value - 'A' + 10;
}

bool ChecksumIsValid(const std::string &sentence) {
  if (sentence.size() < 5 || sentence[0] != '$') return false;
  const std::string::size_type star = sentence.find('*');
  if (star == std::string::npos || star + 2 >= sentence.size()) return false;
  if (!IsHex(sentence[star + 1]) || !IsHex(sentence[star + 2])) return false;

  unsigned char checksum = 0;
  for (std::string::size_type i = 1; i < star; ++i) {
    checksum ^= static_cast<unsigned char>(sentence[i]);
  }
  const int expected = HexValue(sentence[star + 1]) * 16 +
                       HexValue(sentence[star + 2]);
  return checksum == expected;
}

// 去掉校验和部分并按逗号切分字段。空字段必须保留，否则字段下标会错位。
std::vector<std::string> SplitFields(const std::string &sentence) {
  const std::string::size_type star = sentence.find('*');
  const std::string body = sentence.substr(0, star == std::string::npos
                                                   ? sentence.size()
                                                   : star);
  std::vector<std::string> fields;
  std::stringstream stream(body);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  if (!body.empty() && body[body.size() - 1] == ',') fields.push_back("");
  return fields;
}

// 下面两个函数负责把 NMEA 字符串字段转换成数值，并拒绝空值和非法字符。
bool ParseUnsigned(const std::string &value, int *result) {
  if (value.empty() || !result) return false;
  char *end = nullptr;
  const long parsed = std::strtol(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0') return false;
  *result = static_cast<int>(parsed);
  return true;
}

bool ParseDouble(const std::string &value, double *result) {
  if (value.empty() || !result) return false;
  char *end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || !std::isfinite(parsed)) {
    return false;
  }
  *result = parsed;
  return true;
}

// 解析 hhmmss.sss 格式的 UTC 时刻。
bool ParseTime(const std::string &value, UtcTime *utc) {
  if (!utc || value.size() < 6) return false;
  int hour = 0;
  int minute = 0;
  double second = 0.0;
  if (!ParseUnsigned(value.substr(0, 2), &hour) ||
      !ParseUnsigned(value.substr(2, 2), &minute) ||
      !ParseDouble(value.substr(4), &second)) {
    return false;
  }
  if (hour > 23 || minute > 59 || second < 0.0 || second >= 60.0) {
    return false;
  }
  utc->hour = hour;
  utc->minute = minute;
  utc->second = second;
  return true;
}

// 解析 ddmmyy 格式的日期。NMEA 两位年份按 80 年分界转换为四位年份。
bool ParseDate(const std::string &value, UtcTime *utc) {
  if (!utc || value.size() != 6) return false;
  int day = 0;
  int month = 0;
  int year = 0;
  if (!ParseUnsigned(value.substr(0, 2), &day) ||
      !ParseUnsigned(value.substr(2, 2), &month) ||
      !ParseUnsigned(value.substr(4, 2), &year)) {
    return false;
  }
  if (day < 1 || day > 31 || month < 1 || month > 12) return false;
  utc->day = day;
  utc->month = month;
  utc->year = year >= 80 ? 1900 + year : 2000 + year;
  return true;
}

// NMEA 坐标格式是度分，例如纬度 3112.3456、经度 12128.1234。
// direction 决定南纬和西经是否取负号。
bool ParseCoordinate(const std::string &value, const std::string &direction,
                     bool latitude, double *coordinate) {
  if (!coordinate || value.empty() || direction.size() != 1) return false;
  double raw = 0.0;
  if (!ParseDouble(value, &raw)) return false;
  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - degrees * 100.0;
  const double max_degrees = latitude ? 90.0 : 180.0;
  if (degrees < 0 || degrees > max_degrees || minutes < 0.0 || minutes >= 60.0 ||
      (degrees == static_cast<int>(max_degrees) && minutes > 0.0)) {
    return false;
  }
  if ((latitude && direction != "N" && direction != "S") ||
      (!latitude && direction != "E" && direction != "W")) {
    return false;
  }
  *coordinate = degrees + minutes / 60.0;
  if (direction == "S" || direction == "W") *coordinate = -*coordinate;
  return true;
}

// 通过字段首项识别 GGA/RMC，同时兼容 GP 和 GN 两类 talker ID。
bool IsSentenceType(const std::vector<std::string> &fields,
                    const char *type) {
  return !fields.empty() && fields[0].size() == 6 && fields[0][0] == '$' &&
         fields[0].substr(3) == type;
}

// 将时分秒转换为当天经过的秒数，用于 GGA/RMC 配对。
double SecondsOfDay(const UtcTime &utc) {
  return utc.hour * 3600.0 + utc.minute * 60.0 + utc.second;
}

}  // namespace

UtcTime::UtcTime()
    : year(0), month(0), day(0), hour(0), minute(0), second(0.0) {}

RmcData::RmcData()
    : valid(false), has_position(false), latitude(0.0), longitude(0.0) {}

GgaData::GgaData()
    : valid(false),
      latitude(0.0),
      longitude(0.0),
      altitude(std::numeric_limits<double>::quiet_NaN()),
      fix_quality(0),
      satellites(0),
      hdop(std::numeric_limits<double>::quiet_NaN()) {}

bool ParseRmc(const std::string &sentence, RmcData *data) {
  // RMC 的日期和时间是 NavSatFix.header.stamp 的唯一来源。
  if (!data || !ChecksumIsValid(sentence)) return false;
  const std::vector<std::string> fields = SplitFields(sentence);
  if (!IsSentenceType(fields, "RMC") || fields.size() < 10) return false;
  if (!ParseTime(fields[1], &data->utc) || !ParseDate(fields[9], &data->utc)) {
    return false;
  }
  if (fields[2] != "A") return false;

  data->valid = true;
  if (fields.size() >= 7 &&
      ParseCoordinate(fields[3], fields[4], true, &data->latitude) &&
      ParseCoordinate(fields[5], fields[6], false, &data->longitude)) {
    data->has_position = true;
  }
  return true;
}

bool ParseGga(const std::string &sentence, GgaData *data) {
  // GGA 必须有有效时间、经纬度和大于 0 的定位质量，才允许进入发布缓存。
  if (!data || !ChecksumIsValid(sentence)) return false;
  const std::vector<std::string> fields = SplitFields(sentence);
  if (!IsSentenceType(fields, "GGA") || fields.size() < 10) return false;
  if (!ParseTime(fields[1], &data->utc) ||
      !ParseCoordinate(fields[2], fields[3], true, &data->latitude) ||
      !ParseCoordinate(fields[4], fields[5], false, &data->longitude) ||
      !ParseUnsigned(fields[6], &data->fix_quality) || data->fix_quality <= 0) {
    return false;
  }

  ParseUnsigned(fields[7], &data->satellites);
  ParseDouble(fields[8], &data->hdop);
  ParseDouble(fields[9], &data->altitude);
  data->valid = true;
  return true;
}

bool UtcClose(const UtcTime &rmc_time, const UtcTime &gga_time,
              double tolerance_seconds) {
  // 串口读取顺序不能作为配对依据；使用语句内 UTC 时刻进行匹配。
  if (tolerance_seconds < 0.0) return false;
  if (rmc_time.year != 0 && gga_time.year != 0 &&
      (rmc_time.year != gga_time.year || rmc_time.month != gga_time.month ||
       rmc_time.day != gga_time.day)) {
    return false;
  }
  double difference = std::fabs(SecondsOfDay(rmc_time) - SecondsOfDay(gga_time));
  if (difference > 43200.0) difference = 86400.0 - difference;
  return difference <= tolerance_seconds;
}

}  // namespace beiyun_gnss
