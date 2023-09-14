

#include "robot_localization/navsat_conversions.h"

#include <gtest/gtest.h>

#include <string>

void NavsatConversionsTest(const double lat, const double lon,
                           const double UTMNorthing, const double UTMEasting,
                           const std::string UTMZone, const double gamma)
{
  double UTMNorthing_new;
  double UTMEasting_new;
  std::string UTMZone_new;
  double gamma_new;
  RobotLocalization::NavsatConversions::LLtoUTM(lat, lon, UTMNorthing_new, UTMEasting_new, UTMZone_new, gamma_new);
  EXPECT_NEAR(UTMNorthing, UTMNorthing_new, 1e-2);
  EXPECT_NEAR(UTMEasting, UTMEasting_new, 1e-2);
  EXPECT_EQ(UTMZone, UTMZone_new);
  EXPECT_NEAR(gamma, gamma_new, 1e-2);
  double lat_new;
  double lon_new;
  RobotLocalization::NavsatConversions::UTMtoLL(UTMNorthing, UTMEasting, UTMZone, lat_new, lon_new);
  EXPECT_NEAR(lat_new, lat, 1e-5);
  EXPECT_NEAR(lon_new, lon, 1e-5);
}

TEST(NavsatConversionsTest, UtmTest)
{
  NavsatConversionsTest(51.423964, 5.494271, 5699924.709, 673409.989, "31U", 1.950);
  NavsatConversionsTest(-43.530955, 172.636645, 5178919.718, 632246.802, "59G", -1.127);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS( );
}
