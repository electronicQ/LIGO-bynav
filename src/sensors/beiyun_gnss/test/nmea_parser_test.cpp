#include <gtest/gtest.h>

#include <cmath>

#include "beiyun_gnss/covariance_policy.h"
#include "beiyun_gnss/nmea_parser.h"

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace {

const char kRmc[] =
    "$GPRMC,092725.00,A,4717.11399,N,00833.91590,E,0.004,77.52,091202,,,A*54";
const char kGga[] =
    "$GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,0.9,545.4,M,46.9,M,,*67";

}  // namespace

TEST(NmeaParserTest, ParsesCompleteRmcUtc) {
  beiyun_gnss::RmcData rmc;

  ASSERT_TRUE(beiyun_gnss::ParseRmc(kRmc, &rmc));
  EXPECT_TRUE(rmc.valid);
  EXPECT_EQ(2002, rmc.utc.year);
  EXPECT_EQ(12, rmc.utc.month);
  EXPECT_EQ(9, rmc.utc.day);
  EXPECT_EQ(9, rmc.utc.hour);
  EXPECT_EQ(27, rmc.utc.minute);
  EXPECT_NEAR(25.0, rmc.utc.second, 1e-9);
}

TEST(NmeaParserTest, ParsesGgaPositionAndQuality) {
  beiyun_gnss::GgaData gga;

  ASSERT_TRUE(beiyun_gnss::ParseGga(kGga, &gga));
  EXPECT_TRUE(gga.valid);
  EXPECT_NEAR(47.2852331667, gga.latitude, 1e-9);
  EXPECT_NEAR(8.565265, gga.longitude, 1e-9);
  EXPECT_NEAR(545.4, gga.altitude, 1e-9);
  EXPECT_EQ(1, gga.fix_quality);
  EXPECT_EQ(8, gga.satellites);
  EXPECT_NEAR(0.9, gga.hdop, 1e-9);
}

TEST(NmeaParserTest, RejectsBadChecksumAndInvalidFix) {
  beiyun_gnss::RmcData rmc;
  beiyun_gnss::GgaData gga;

  EXPECT_FALSE(beiyun_gnss::ParseRmc(
      "$GPRMC,092725.00,A,4717.11399,N,00833.91590,E,0.004,77.52,091202,,,A*55",
      &rmc));
  EXPECT_FALSE(beiyun_gnss::ParseRmc(
      "$GPRMC,,V,,,,,,,,,,N*53", &rmc));
  EXPECT_FALSE(beiyun_gnss::ParseGga(
      "$GPGGA,,,,,,,,,,,,,,*56", &gga));
}

TEST(NmeaParserTest, MatchesGgaAndRmcWithinConfiguredTolerance) {
  beiyun_gnss::RmcData rmc;
  beiyun_gnss::GgaData gga;

  ASSERT_TRUE(beiyun_gnss::ParseRmc(kRmc, &rmc));
  ASSERT_TRUE(beiyun_gnss::ParseGga(kGga, &gga));
  gga.utc.second += 0.005;
  EXPECT_TRUE(beiyun_gnss::UtcClose(rmc.utc, gga.utc, 0.01));
  EXPECT_FALSE(beiyun_gnss::UtcClose(rmc.utc, gga.utc, 0.001));
}

TEST(CovariancePolicyTest, SelectsConfiguredCovarianceForEachFixQuality) {
  beiyun_gnss::CovarianceConfig config;
  config.single_point_covariance_m2 = 25.0;
  config.dgps_covariance_m2 = 1.0;
  config.rtk_fixed_covariance_m2 = 0.0004;
  config.rtk_float_covariance_m2 = 0.04;

  EXPECT_DOUBLE_EQ(25.0,
                   beiyun_gnss::CovarianceForFixQuality(1, config));
  EXPECT_DOUBLE_EQ(1.0,
                   beiyun_gnss::CovarianceForFixQuality(2, config));
  EXPECT_DOUBLE_EQ(0.0004,
                   beiyun_gnss::CovarianceForFixQuality(4, config));
  EXPECT_DOUBLE_EQ(0.04,
                   beiyun_gnss::CovarianceForFixQuality(5, config));
  EXPECT_DOUBLE_EQ(0.0,
                   beiyun_gnss::CovarianceForFixQuality(0, config));
}

TEST(CovariancePolicyTest, RejectsNegativeConfiguredCovariance) {
  beiyun_gnss::CovarianceConfig config;
  config.rtk_fixed_covariance_m2 = -0.1;

  EXPECT_DOUBLE_EQ(0.0,
                   beiyun_gnss::CovarianceForFixQuality(4, config));
}
