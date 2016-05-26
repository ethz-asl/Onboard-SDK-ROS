#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace DJI::onboardSDK;

int main(int argc, char **argv)
{
  int temp32;
  std::vector<double> easting;
  std::vector<double> northing;
  std::vector<double> height;
  std::vector<double> waiting_time;
  std::vector<double> heading;

  ros::init(argc, argv, "waypoint_navigator");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("easting", easting);
  private_nh.getParam("northing", northing);
  private_nh.getParam("height", height);
  private_nh.getParam("waiting_time", waiting_time);
  private_nh.getParam("heading", heading);
  DJIDrone* drone = new DJIDrone(nh);

	//waypoint action test data
  dji_sdk::WaypointList newWaypointList;
  dji_sdk::Waypoint waypoint;

  // Create list of waypoints to visit.
  for (int i = 0; i < easting.size(); i++) {
    waypoint.latitude = northing[i];
    waypoint.longitude = easting[i];
    waypoint.altitude = height[i];
    waypoint.staytime = waiting_time[i];
    waypoint.heading = heading[i];
    newWaypointList.waypoint_list.push_back(waypoint);
  }

  drone->request_sdk_permission_control();
  printf("Control requested.\n");

  while (1) {

    printf("Press 'a' to take-off.\n");
    printf("Press 'b' to start the mission.\n");
    ros::spinOnce();
    temp32 = getchar();

    if (temp32 == 'a') {
      drone->takeoff();
    }
    else if (temp32 == 'b') {
      drone->waypoint_navigation_send_request(newWaypointList);
    }
  }
  return 0;
}
