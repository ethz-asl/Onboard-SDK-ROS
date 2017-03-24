#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/dji_sdk_node.h>
#include <cstdlib>

using namespace DJI::onboardSDK;

static void Display_Main_Menu(void)
{
  printf("Press 'a' to take-off.\n");
  printf("Press 'b' to start the mission.\n");
}

void rcCallback(const dji_sdk::RCChannels& rcMsg)
{
    // update rollpitchyawratethrust message
    rpyrthMsg.roll = rcMsg.roll;
    rpyrthMsg.pitch = rcMsg.pitch;
    rpyrthMsg.yaw_rate = rcMsg.yaw;
    rpyrthMsg.thrust.z =rcMsg.throttle;
    // RPYRTH control in body frame binary 0b00101011
    drone->attitude_control(0x2B, rpyrthMsg.roll, rpyrthMsg.pitch, rpyrthMsg.yaw_rate,rpyrthMsg.thrust.z);

}

int main(int argc, char **argv)
{
  int keyboard_input;

  // Load parameters.
  ros::init(argc, argv, "dji_remote_control");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  DJIDrone* drone = new DJIDrone(nh);

   /* Register Subscribers */
    ros::Subscriber rc_sub = nh.subscribe("dji_sdk/rc_channels", 1, rcCallback);

  drone->request_sdk_permission_control();
  printf("Control requested.\n");
  drone->mission_waypoint_upload(waypoint_task);
  printf("Mission uploaded.\n");

  Display_Main_Menu();

  while (1) {

    printf("Press 'a' to take-off.\n");
    printf("Press 'b' to start the mission.\n");
    ros::spinOnce();
    keyboard_input = getchar();

    if (keyboard_input == 'a') {
      drone->takeoff();
    }
    else if (keyboard_input == 'b') {
      drone->mission_start();
    }

    Display_Main_Menu();
    
  }
  return 0;
}
