#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <std_srvs/Empty.h>
#include <dji_waypoint_navigator/GoToGlobalWaypoint.h>
#include <dji_waypoint_navigator/GoToLocalWaypoint.h>
#include <dji_waypoint_navigator/GoToExternalWaypoint.h>

using namespace DJI::onboardSDK;

class DJITestNavigatorNode
{
public:
  DJITestNavigatorNode();
  ~DJITestNavigatorNode();

private:

  // Navigation mode services
  bool takeoff(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool go_to_global_waypoint(dji_waypoint_navigator::GoToGlobalWaypoint::Request& request,
                             dji_waypoint_navigator::GoToGlobalWaypoint::Response& response);
  bool go_to_local_waypoint(dji_waypoint_navigator::GoToLocalWaypoint::Request& request,
                           dji_waypoint_navigator::GoToLocalWaypoint::Response& response);
  bool go_to_external_waypoint(dji_waypoint_navigator::GoToExternalWaypoint::Request& request,
                           dji_waypoint_navigator::GoToExternalWaypoint::Response& response);

  DJIDrone* drone_;
  ros::ServiceServer takeoff_service_;
  ros::ServiceServer go_to_global_waypoint_service_;
  ros::ServiceServer go_to_local_waypoint_service_;
  ros::ServiceServer go_to_external_waypoint_service_;
};

DJITestNavigatorNode::DJITestNavigatorNode() {
  ros::NodeHandle nh("");
  drone_ = new DJIDrone(nh);
  drone_->request_sdk_permission_control();
  printf("Control requested.\n");
  takeoff_service_ = nh.advertiseService("takeoff", &DJITestNavigatorNode::takeoff, this);
  go_to_global_waypoint_service_ = nh.advertiseService("go_to_global_waypoint", &DJITestNavigatorNode::go_to_global_waypoint, this);
  go_to_local_waypoint_service_ = nh.advertiseService("go_to_local_waypoint", &DJITestNavigatorNode::go_to_local_waypoint, this);
  go_to_external_waypoint_service_ = nh.advertiseService("go_to_external_waypoint", &DJITestNavigatorNode::go_to_external_waypoint, this);
}

DJITestNavigatorNode::~DJITestNavigatorNode() {
}

bool DJITestNavigatorNode::takeoff(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  drone_->takeoff();
  return true;
}

bool DJITestNavigatorNode::go_to_global_waypoint(dji_waypoint_navigator::GoToGlobalWaypoint::Request& request,
                           dji_waypoint_navigator::GoToGlobalWaypoint::Response& response)
{
  drone_->local_position_navigation_stop_tracking_goal();
  drone_->global_position_navigation_stop_tracking_goal();
  drone_->global_position_navigation_send_request(request.point.x, request.point.y, request.point.z);
  return true;
}

bool DJITestNavigatorNode::go_to_local_waypoint(dji_waypoint_navigator::GoToLocalWaypoint::Request& request,
                         dji_waypoint_navigator::GoToLocalWaypoint::Response& response)
{
  drone_->local_position_navigation_stop_tracking_goal();
  drone_->global_position_navigation_stop_tracking_goal();
  drone_->local_position_navigation_send_request(request.point.x, request.point.y, request.point.z);
  return true;
}

bool DJITestNavigatorNode::go_to_external_waypoint(dji_waypoint_navigator::GoToExternalWaypoint::Request& request,
                         dji_waypoint_navigator::GoToExternalWaypoint::Response& response)
{
  drone_->local_position_navigation_stop_tracking_goal();
  drone_->global_position_navigation_stop_tracking_goal();
  drone_->external_position_navigation_send_request(request.point.x, request.point.y, request.point.z);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_navigator");
  ros::NodeHandle nh;
  DJITestNavigatorNode dji_test_navigator_node;
  ros::spin();
  return 0;
}
