#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace DJI::onboardSDK;

static void Display_Main_Menu(void)
{
  printf("Press 'a' to take-off.\n");
  printf("Press 'b' to start the mission.\n");
}

int main(int argc, char **argv)
{
  int temp32;
  double velocity_range;
  std::vector<double> easting;
  std::vector<double> northing;
  std::vector<double> height;
  std::vector<double> waiting_time;
  std::vector<double> heading;

  // Load parameters.
  ros::init(argc, argv, "waypoint_navigator");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("velocity_range", velocity_range);
  private_nh.getParam("easting", easting);
  private_nh.getParam("northing", northing);
  private_nh.getParam("height", height);
  private_nh.getParam("waiting_time", waiting_time);
  private_nh.getParam("heading", heading);
  DJIDrone* drone = new DJIDrone(nh);

  dji_sdk::MissionWaypointTask waypoint_task;
	dji_sdk::MissionWaypoint waypoint;

  waypoint_task.velocity_range = velocity_range;
  waypoint_task.idle_velocity = 3;
  waypoint_task.action_on_finish = 0;
  waypoint_task.mission_exec_times = 1;
  waypoint_task.yaw_mode = 4;
  waypoint_task.trace_mode = 0;
  waypoint_task.action_on_rc_lost = 0;
  waypoint_task.gimbal_pitch_mode = 0;

  // Create list of waypoints to visit.
  for (int i = 0; i < easting.size(); i++) {
    waypoint.latitude = northing[i];
    waypoint.longitude = easting[i];
    waypoint.altitude = height[i];
    waypoint.damping_distance = 0;
    waypoint.target_yaw = 0;
    waypoint.target_gimbal_pitch = 0;
    waypoint.turn_mode = 0;
    waypoint.has_action = 0;
    waypoint_task.mission_waypoint.push_back(waypoint);
  }

  drone->request_sdk_permission_control();
  printf("Control requested.\n");
  drone->mission_waypoint_upload(waypoint_task);
  printf("Mission uploaded.\n");

  Display_Main_Menu();

  while (1) {

    printf("Press 'a' to take-off.\n");
    printf("Press 'b' to start the mission.\n");
    ros::spinOnce();
    temp32 = getchar();

    if (temp32 == 'a') {
      drone->takeoff();
    }
    else if (temp32 == 'b') {
      drone->mission_start();
    }

    Display_Main_Menu();
    
  }
  return 0;
}
