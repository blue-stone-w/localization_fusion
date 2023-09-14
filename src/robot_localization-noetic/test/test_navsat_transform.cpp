

#include "robot_localization/navsat_transform.h"
#include <robot_localization/SetDatum.h>
#include <robot_localization/ToLL.h>
#include <robot_localization/FromLL.h>

#include <gtest/gtest.h>

#include <string>

TEST(NavSatTransformUTMJumpTest, UtmTest)
{
  ros::NodeHandle nh;
  ros::ServiceClient set_datum_client = nh.serviceClient<robot_localization::SetDatum>("/datum");
  ros::ServiceClient from_ll_client   = nh.serviceClient<robot_localization::FromLL>("/fromLL");

  EXPECT_TRUE(set_datum_client.waitForExistence(ros::Duration(5)));

  // Initialise the navsat_transform node to a UTM zone
  robot_localization::SetDatum set_datum_srv;
  set_datum_srv.request.geo_pose.position.latitude  = 1;
  set_datum_srv.request.geo_pose.position.longitude = 4;
  set_datum_srv.request.geo_pose.orientation.w      = 1;
  EXPECT_TRUE(set_datum_client.call(set_datum_srv));

  // Let the node figure out its transforms
  ros::Duration(0.2).sleep( );

  // Request the GPS point of said point:
  robot_localization::FromLL from_ll_srv;
  from_ll_srv.request.ll_point.latitude  = 10;
  from_ll_srv.request.ll_point.longitude = 4.5;
  EXPECT_TRUE(from_ll_client.call(from_ll_srv));
  auto initial_response = from_ll_srv.response;

  // Request GPS point also in that zone
  from_ll_srv.request.ll_point.longitude = 5.5;
  EXPECT_TRUE(from_ll_client.call(from_ll_srv));
  auto same_zone_response = from_ll_srv.response;

  // 1° of longitude is about 111 kilometers in length
  EXPECT_NEAR(initial_response.map_point.x, same_zone_response.map_point.x, 120e3);
  EXPECT_NEAR(initial_response.map_point.y, same_zone_response.map_point.y, 120e3);

  // Request GPS point from neighboring zone (zone crossing is at 6 degrees)
  from_ll_srv.request.ll_point.longitude = 6.5;
  from_ll_client.call(from_ll_srv);
  auto neighbour_zone_response = from_ll_srv.response;

  EXPECT_NEAR(initial_response.map_point.x, neighbour_zone_response.map_point.x, 2 * 120e3);
  EXPECT_NEAR(initial_response.map_point.y, neighbour_zone_response.map_point.y, 2 * 120e3);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_navsat_transform");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS( );
}
