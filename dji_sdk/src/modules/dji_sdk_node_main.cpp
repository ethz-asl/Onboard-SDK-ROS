/** @file dji_sdk_node_main.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  Broadcast and Mobile callbacks are implemented here. 
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>
#include <functional>
#include <dji_sdk/DJI_LIB_ROS_Adapter.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Transform.h>
#include <iomanip>
#include <cuckoo_time_translator/DeviceTimeTranslator.h>
#include <stdint.h>

//----------------------------------------------------------
// timer spin_function 50Hz
//----------------------------------------------------------
using namespace std;

void DJISDKNode::transparent_transmission_callback(uint8_t *buf, uint8_t len)
{
    dji_sdk::TransparentTransmissionData transparent_transmission_data;
    transparent_transmission_data.data.resize(len);
    memcpy(&transparent_transmission_data.data[0], buf, len);
    data_received_from_remote_device_publisher.publish(transparent_transmission_data);
}
void DJISDKNode::broadcast_callback()
{
    DJI::onboardSDK::BroadcastData bc_data = rosAdapter->coreAPI->getBroadcastData();
    DJI::onboardSDK::Version version = rosAdapter->coreAPI->getSDKVersion();
    unsigned short msg_flags = bc_data.dataFlag;

    static int frame_id = 0;
    frame_id++;

    auto current_time = ros::Time::now();

    if (msg_flags & HAS_TIME)
    {

        // Only this message containes the received time in the header and the
        // hardware time within the message
        // All following messages should have translated hardware times in the
        // header.

        time_stamp.header.frame_id = "/time";
        time_stamp.header.stamp = current_time;
        time_stamp.time = bc_data.timeStamp.time; // This is now in units of 1/400
                                                  // s but resolution is only of
                                                  // the order of 1s!!!
        time_stamp.time_ns =
            bc_data.timeStamp.nanoTime; // This is perhaps more accurate??? But
                                        // requires handling the wrapping after
                                        // every 4.295 secs. DO NOT SUM both as it
                                        // leads to double counting.

        time_stamp.sync_flag = bc_data.timeStamp.syncFlag;
        time_stamp_publisher.publish(time_stamp);

        // Use translated hardware time
        if (device_time_translator_)
        {
            current_time = device_time_translator_->update(bc_data.timeStamp.nanoTime, current_time, 0.0);
        }
        /*
      if (device_time_translator_->isReadyToTranslate()) {
       current_time =
            device_time_translator_->translate(bc_data.timeStamp.nanoTime);
      }*/ else
        {
            ROS_WARN(
                "device_time_translator is not ready yet, using ros::Time::now()");
        }
    }

    // update Imu message
    if ((msg_flags & HAS_Q) && (msg_flags & HAS_W) && (msg_flags & HAS_A))
    {

        ROS_DEBUG_STREAM("Frame " << frame_id << " "
                                  << "Hardware timeStamp.nanoTime " << std::setprecision(15) << double(bc_data.timeStamp.nanoTime) * 1e-9);
        imu_msg.header.frame_id = "/world";

        imu_msg.header.stamp = current_time;
        //TODO: Raghav - Update with nearest neighbour messages to increase frequency
        // Conversion from NED to NWU (XYZ--robot body frame)
        /*
      tf::Quaternion qNED2ENU();
      qNED2ENU.setEulerZYX(pi/2,0, -pi/2);
      tf::Quaternion qIMU(bc_data.q.q1, bc_data.q.q2, bc_data.q.q3,
      bc_data.q.q0);
      tf::Quaternion qIMUENU;
      qIMUENU = qNED2ENU*qIMU;
      imu_msg.orientation = quaternionTFToMsg(qIMUENU);
      */
        // Convert from NED to ENU frame // This is conversion to NWU frame not
        // ENU
        imu_msg.orientation.w = bc_data.q.q0;
        imu_msg.orientation.x = bc_data.q.q1;
        imu_msg.orientation.y = -bc_data.q.q2;
        imu_msg.orientation.z = -bc_data.q.q3;
        imu_msg.angular_velocity.x = bc_data.w.x;
        imu_msg.angular_velocity.y = -bc_data.w.y;
        imu_msg.angular_velocity.z = -bc_data.w.z;

        imu_msg.linear_acceleration.x = bc_data.a.x * 9.807;
        imu_msg.linear_acceleration.y = -bc_data.a.y * 9.807;
        imu_msg.linear_acceleration.z = -bc_data.a.z * 9.807;

        imu_msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0};
        imu_msg.angular_velocity_covariance = {0, 0.0, 0.0, 0.0, 0,
                                               0.0, 0.0, 0.0, 0};
        imu_msg.linear_acceleration_covariance = {0, 0.0, 0.0, 0.0, 0,
                                                  0.0, 0.0, 0.0, 0};
        imu_msg_publisher.publish(imu_msg);
    }

    //update attitude msg
    if ((msg_flags & HAS_Q) && (msg_flags & HAS_W))
    {
        attitude_quaternion.header.frame_id = "/world";
        attitude_quaternion.header.stamp = current_time;
        attitude_quaternion.q0 = bc_data.q.q0;
        attitude_quaternion.q1 = bc_data.q.q1;
        attitude_quaternion.q2 = bc_data.q.q2;
        attitude_quaternion.q3 = bc_data.q.q3;
        attitude_quaternion.wx = bc_data.w.x;
        attitude_quaternion.wy = bc_data.w.y;
        attitude_quaternion.wz = bc_data.w.z;
        attitude_quaternion.ts = bc_data.timeStamp.time;
        attitude_quaternion_publisher.publish(attitude_quaternion);
    }

    //update global_position, gps and geopose msg
    if (msg_flags & HAS_POS)
    {
        global_position.header.frame_id = "/world";
        global_position.header.stamp = current_time;
        global_position.ts = bc_data.timeStamp.time;
        global_position.latitude = bc_data.pos.latitude * 180.0 / C_PI;
        global_position.longitude = bc_data.pos.longitude * 180.0 / C_PI;
        global_position.height = bc_data.pos.height;
        global_position.altitude = bc_data.pos.altitude;
        global_position.health = bc_data.pos.health;
        global_position_publisher.publish(global_position);

        // Update gps mesage
        gps_msg.header.frame_id = "/world";
        gps_msg.header.stamp = current_time;
        gps_msg.latitude = bc_data.pos.latitude * 180.0 / C_PI;
        gps_msg.longitude = bc_data.pos.longitude * 180.0 / C_PI;
        gps_msg.altitude = bc_data.pos.altitude;
        gps_msg_publisher.publish(gps_msg);

        //TODO:
        // FIX BUG about flying at lat = 0
        if (global_position.ts != 0 && global_position_ref_seted == 0 && global_position.latitude != 0 && global_position.health > 3)
        {
            global_position_ref = global_position;
            global_position_ref_seted = 1;
        }

        //update local_position msg
        local_position.header.frame_id = "/world";
        local_position.header.stamp = current_time;
        gps_convert_ned(
            local_position.x,
            local_position.y,
            global_position.longitude,
            global_position.latitude,
            global_position_ref.longitude,
            global_position_ref.latitude);
        local_position.z = global_position.height;
        local_position.ts = global_position.ts;
        local_position_ref = local_position;
        local_position_publisher.publish(local_position);
    }

    //update velocity msg
    if (msg_flags & HAS_V)
    {
        velocity.header.frame_id = "/world";
        velocity.header.stamp = current_time;
        velocity.ts = bc_data.timeStamp.time;
        velocity.vx = bc_data.v.x;
        velocity.vy = bc_data.v.y;
        velocity.vz = bc_data.v.z;
        velocity.health_flag = bc_data.v.health;
        velocity_publisher.publish(velocity);
    }

    //update acceleration msg
    if (msg_flags & HAS_A)
    {
        acceleration.header.frame_id = "/world";
        acceleration.header.stamp = current_time;
        acceleration.ts = bc_data.timeStamp.time;
        acceleration.ax = bc_data.a.x;
        acceleration.ay = bc_data.a.y;
        acceleration.az = bc_data.a.z;
        acceleration_publisher.publish(acceleration);
    }

    //update gimbal msg
    if (msg_flags & HAS_GIMBAL)
    {
        gimbal.header.frame_id = "/gimbal";
        gimbal.header.stamp = current_time;
        gimbal.ts = bc_data.timeStamp.time;
        gimbal.roll = bc_data.gimbal.roll;
        gimbal.pitch = bc_data.gimbal.pitch;
        gimbal.yaw = bc_data.gimbal.yaw;
        gimbal_publisher.publish(gimbal);
    }

    //update odom msg
    if ((msg_flags & HAS_POS) && (msg_flags & HAS_Q) && (msg_flags & HAS_W) && (msg_flags & HAS_V))
    {
        //odometry.header.frame_id = "/world";
        odometry.header.frame_id = "/odom";
        odometry.child_frame_id = "/base_link";

        odometry.header.stamp = current_time;
        // Convert from NED to NWU
        /*
        odometry.pose.pose.position.x = local_position.x;
        odometry.pose.pose.position.y = -local_position.y;
        odometry.pose.pose.position.z = local_position.z;
        */
        odometry.pose.pose.orientation.w = attitude_quaternion.q0;
        odometry.pose.pose.orientation.x = attitude_quaternion.q1;
        odometry.pose.pose.orientation.y = -attitude_quaternion.q2;
        odometry.pose.pose.orientation.z = -attitude_quaternion.q3;

        // Convert from NED to ENU frame
        odometry.pose.pose.position.x = local_position.y;
        //odometry.pose.pose.position.y = local_position.y;
        odometry.pose.pose.position.y = local_position.x;
        odometry.pose.pose.position.z = local_position.z;
        tf::Quaternion qOdom;
        tf::quaternionMsgToTF(odometry.pose.pose.orientation, qOdom);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(M_PI / 2) * qOdom, odometry.pose.pose.orientation);
        odometry.twist.twist.angular.x = attitude_quaternion.wy;
        odometry.twist.twist.angular.y = attitude_quaternion.wx;
        odometry.twist.twist.angular.z = -attitude_quaternion.wz;
        odometry.twist.twist.linear.x = velocity.vy;
        odometry.twist.twist.linear.y = velocity.vx;
        odometry.twist.twist.linear.z = velocity.vz;

        static tf::TransformBroadcaster broad;
        tf::Transform trans;
        trans.setOrigin(tf::Vector3(odometry.pose.pose.position.x,
                                    odometry.pose.pose.position.y,
                                    odometry.pose.pose.position.z));
        tf::Quaternion q(odometry.pose.pose.orientation.x,
                         odometry.pose.pose.orientation.y,
                         odometry.pose.pose.orientation.z,
                         odometry.pose.pose.orientation.w);
        tf::Vector3 v_world(odometry.twist.twist.linear.x,
                            odometry.twist.twist.linear.y,
                            odometry.twist.twist.linear.z);

        tf::Vector3 v_body = v_world.rotate(q.getAxis(), -q.getAngle());
        odometry.twist.twist.linear.x = v_body.x();
        odometry.twist.twist.linear.y = v_body.y();
        odometry.twist.twist.linear.z = v_body.z();

        trans.setRotation(q);
        broad.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "odom", "base_link"));
        odometry_publisher.publish(odometry);

        // Update geopose msg
        geopose_msg.header.stamp = current_time;
        geopose_msg.header.frame_id = "/global_enu";
        geopose_msg.pose.position.latitude = bc_data.pos.latitude * 180.0 / C_PI;
        geopose_msg.pose.position.longitude = bc_data.pos.longitude * 180.0 / C_PI;
        geopose_msg.pose.position.altitude = bc_data.pos.altitude;
        geopose_msg.pose.orientation = odometry.pose.pose.orientation;
        geopose_msg_publisher.publish(geopose_msg);
    }

    //update rc_channel msg
    if (msg_flags & HAS_RC)
    {
        rc_channels.header.frame_id = "/rc";
        rc_channels.header.stamp = current_time;
        rc_channels.ts = bc_data.timeStamp.time;
        rc_channels.pitch = bc_data.rc.pitch;
        rc_channels.roll = bc_data.rc.roll;
        rc_channels.mode = bc_data.rc.mode;
        rc_channels.gear = bc_data.rc.gear;
        rc_channels.throttle = bc_data.rc.throttle;
        rc_channels.yaw = bc_data.rc.yaw;
        rc_channels_publisher.publish(rc_channels);
    }

    //update compass msg
    if (msg_flags & HAS_MAG)
    {
        compass.header.frame_id = "/world";
        compass.header.stamp = current_time;
        compass.ts = bc_data.timeStamp.time;
        compass.x = bc_data.mag.x;
        compass.y = bc_data.mag.y;
        compass.z = bc_data.mag.z;
        compass_publisher.publish(compass);
    }
    /*
    //update odom msg
    if ( (msg_flags & HAS_POS) && (msg_flags & HAS_Q) && (msg_flags & HAS_W) && (msg_flags & HAS_V) ) {
        odometry.header.frame_id = "/world";
        odometry.header.stamp = current_time;
        odometry.pose.pose.position.x = local_position.x;
        odometry.pose.pose.position.y = local_position.y;
        odometry.pose.pose.position.z = local_position.z;
        odometry.pose.pose.orientation.w = attitude_quaternion.q0;
        odometry.pose.pose.orientation.x = attitude_quaternion.q1;
        odometry.pose.pose.orientation.y = attitude_quaternion.q2;
        odometry.pose.pose.orientation.z = attitude_quaternion.q3;
        odometry.twist.twist.angular.x = attitude_quaternion.wx;
        odometry.twist.twist.angular.y = attitude_quaternion.wy;
        odometry.twist.twist.angular.z = attitude_quaternion.wz;
        odometry.twist.twist.linear.x = velocity.vx;
        odometry.twist.twist.linear.y = velocity.vy;
        odometry.twist.twist.linear.z = velocity.vz;
        odometry_publisher.publish(odometry);
    }
*/

    /******************************************************************
****************************If using A3****************************
******************************************************************/

    if (version == DJI::onboardSDK::versionA3_31 || DJI::onboardSDK::versionA3_32)
    {

        //update gimbal msg
        if (msg_flags & A3_HAS_GIMBAL)
        {
            gimbal.header.frame_id = "/gimbal";
            gimbal.header.stamp = current_time;
            gimbal.ts = bc_data.timeStamp.time;
            gimbal.roll = bc_data.gimbal.roll;
            gimbal.pitch = bc_data.gimbal.pitch;
            gimbal.yaw = bc_data.gimbal.yaw;
            gimbal_publisher.publish(gimbal);
        }

        //update rc_channel msg
        if (msg_flags & A3_HAS_RC)
        {
            rc_channels.header.frame_id = "/rc";
            rc_channels.header.stamp = current_time;
            rc_channels.ts = bc_data.timeStamp.time;
            rc_channels.pitch = bc_data.rc.pitch;
            rc_channels.roll = bc_data.rc.roll;
            rc_channels.mode = bc_data.rc.mode;
            rc_channels.gear = bc_data.rc.gear;
            rc_channels.throttle = bc_data.rc.throttle;
            rc_channels.yaw = bc_data.rc.yaw;
            rc_channels_publisher.publish(rc_channels);
        }

        if (msg_flags & A3_HAS_GPS)
        {
            A3_GPS.date = bc_data.gps.date;
            A3_GPS.time = bc_data.gps.time;
            A3_GPS.longitude = bc_data.gps.longitude;
            A3_GPS.latitude = bc_data.gps.latitude;
            A3_GPS.height_above_sea = bc_data.gps.Hmsl;
            A3_GPS.velocity_north = bc_data.gps.velocityNorth;
            A3_GPS.velocity_east = bc_data.gps.velocityEast;
            A3_GPS.velocity_ground = bc_data.gps.velocityGround;
            A3_GPS_info_publisher.publish(A3_GPS);
        }
        if (msg_flags & A3_HAS_RTK)
            A3_RTK.date = bc_data.rtk.date;
        A3_RTK.time = bc_data.rtk.time;
        A3_RTK.longitude_RTK = bc_data.rtk.longitude;
        A3_RTK.latitude_RTK = bc_data.rtk.latitude;
        A3_RTK.height_above_sea_RTK = bc_data.rtk.Hmsl;
        A3_RTK.velocity_north = bc_data.rtk.velocityNorth;
        A3_RTK.velocity_east = bc_data.rtk.velocityEast;
        A3_RTK.velocity_ground = bc_data.rtk.velocityGround;
        A3_RTK.yaw = bc_data.rtk.yaw;
        A3_RTK.position_flag = bc_data.rtk.posFlag;
        A3_RTK.yaw_flag = bc_data.rtk.yawFlag;
        A3_RTK_info_publisher.publish(A3_RTK);

        //update compass msg
        if (msg_flags & A3_HAS_MAG)
        {
            compass.header.frame_id = "/world";
            compass.header.stamp = current_time;
            compass.ts = bc_data.timeStamp.time;
            compass.x = bc_data.mag.x;
            compass.y = bc_data.mag.y;
            compass.z = bc_data.mag.z;
            compass_publisher.publish(compass);
        }

        //update flight_status
        if (msg_flags & A3_HAS_STATUS)
        {
            std_msgs::UInt8 msg;
            flight_status = bc_data.status;
            msg.data = flight_status;
            flight_status_publisher.publish(msg);
        }

        //update battery msg
        if (msg_flags & A3_HAS_BATTERY)
        {
            power_status.percentage = bc_data.battery;
            power_status_publisher.publish(power_status);
        }

        //update flight control info
        if (msg_flags & A3_HAS_DEVICE)
        {
            flight_control_info.control_mode = bc_data.ctrlInfo.mode;
            flight_control_info.cur_ctrl_dev_in_navi_mode = bc_data.ctrlInfo.deviceStatus;
            flight_control_info.serial_req_status = bc_data.ctrlInfo.flightStatus;
            flight_control_info.virtual_rc_status = bc_data.ctrlInfo.vrcStatus;
            flight_control_info_publisher.publish(flight_control_info);
        }
    }

    /******************************************************************
***************************If using M100***************************
******************************************************************/

    else
    {

        if (msg_flags & HAS_GIMBAL)
        {
            gimbal.header.frame_id = "/gimbal";
            gimbal.header.stamp = current_time;
            gimbal.ts = bc_data.timeStamp.time;
            gimbal.roll = bc_data.gimbal.roll;
            gimbal.pitch = bc_data.gimbal.pitch;
            gimbal.yaw = bc_data.gimbal.yaw;
            gimbal_publisher.publish(gimbal);
        }

        //update rc_channel msg
        if (msg_flags & HAS_RC)
        {
            rc_channels.header.frame_id = "/rc";
            rc_channels.header.stamp = current_time;
            rc_channels.ts = bc_data.timeStamp.time;
            rc_channels.pitch = bc_data.rc.pitch;
            rc_channels.roll = bc_data.rc.roll;
            rc_channels.mode = bc_data.rc.mode;
            rc_channels.gear = bc_data.rc.gear;
            rc_channels.throttle = bc_data.rc.throttle;
            rc_channels.yaw = bc_data.rc.yaw;
            rc_channels_publisher.publish(rc_channels);
        }

        //update compass msg
        if (msg_flags & HAS_MAG)
        {
            compass.header.frame_id = "/world";
            compass.header.stamp = current_time;
            compass.ts = bc_data.timeStamp.time;
            compass.x = bc_data.mag.x;
            compass.y = bc_data.mag.y;
            compass.z = bc_data.mag.z;
            compass_publisher.publish(compass);
        }

        //update flight_status
        if (msg_flags & HAS_STATUS)
        {
            std_msgs::UInt8 msg;
            flight_status = bc_data.status;
            msg.data = flight_status;
            flight_status_publisher.publish(msg);
        }

        //update battery msg
        if (msg_flags & HAS_BATTERY)
        {
            power_status.percentage = bc_data.battery;
            power_status_publisher.publish(power_status);
        }

        //update flight control info
        if (msg_flags & HAS_DEVICE)
        {
            flight_control_info.control_mode = bc_data.ctrlInfo.mode;
            flight_control_info.cur_ctrl_dev_in_navi_mode = bc_data.ctrlInfo.deviceStatus;
            flight_control_info.serial_req_status = bc_data.ctrlInfo.flightStatus;
            flight_control_info.virtual_rc_status = bc_data.ctrlInfo.vrcStatus;
            flight_control_info_publisher.publish(flight_control_info);
        }
    }

    //update obtaincontrol msg
    std_msgs::UInt8 msg;
    activation_result = bc_data.activation;
    msg.data = bc_data.activation;
    activation_publisher.publish(msg);
}

int DJISDKNode::init_parameters(ros::NodeHandle &nh_private)
{
    std::string drone_version;
    std::string serial_name;
    int baud_rate;
    int app_id;
    int app_version;
    std::string app_bundle_id; //reserved
    std::string enc_key;
    int uart_or_usb;

    nh_private.param("serial_name", serial_name, std::string("/dev/ttyTHS1"));
    //nh_private.param("baud_rate", baud_rate, 230400);
    nh_private.param("baud_rate", baud_rate, 921600);

    nh_private.param("app_id", app_id, 1022384);
    nh_private.param("app_version", app_version, 1);
    nh_private.param("enc_key", enc_key,
                     std::string("e7bad64696529559318bb35d0a8c6050d3b88e791e1808cfe8f7802150ee6f0d"));

    nh_private.param("uart_or_usb", uart_or_usb, 0);                       //chosse uart as default
    nh_private.param("drone_version", drone_version, std::string("M100")); //chosse M100 as default

    // activation
    user_act_data.ID = app_id;

    if ((uart_or_usb) && (drone_version.compare("M100")))
    {
        printf("M100 does not support USB API.\n");
        return -1;
    }

    if (!drone_version.compare("M100"))
    {
        user_act_data.version = versionM100_31;
    }
    else if (!drone_version.compare("A3_31"))
    {
        user_act_data.version = versionA3_31;
    }
    else if (!drone_version.compare("A3_32"))
    {
        user_act_data.version = versionA3_32;
    }
    user_act_data.encKey = app_key;
    strcpy(user_act_data.encKey, enc_key.c_str());

    printf("=================================================\n");
    printf("app id: %d\n", user_act_data.ID);
    printf("app version: 0x0%X\n", user_act_data.version);
    printf("app key: %s\n", user_act_data.encKey);
    printf("=================================================\n");

    if (uart_or_usb) //use usb port for SDK
    {
        rosAdapter->usbHandshake(serial_name);
    }

    rosAdapter->init(serial_name, baud_rate);
    rosAdapter->activate(&user_act_data, NULL);
    rosAdapter->setBroadcastCallback(&DJISDKNode::broadcast_callback, this);
    rosAdapter->setFromMobileCallback(&DJISDKNode::transparent_transmission_callback, this);

    return 0;
}

DJISDKNode::DJISDKNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private) : dji_sdk_mission(nullptr)
{

    init_publishers(nh);
    init_services(nh);
    init_actions(nh);
    init_subscribers(nh);
    // Init time stamp translator
    device_time_translator_.reset(new cuckoo_time_translator::DefaultDeviceTimeUnwrapperAndTranslator(
        cuckoo_time_translator::WrappingClockParameters(UINT32_MAX, 1e9),
        nh.getNamespace(), cuckoo_time_translator::Defaults().setFilterAlgorithm(cuckoo_time_translator::FilterAlgorithm::ConvexHull)));

    init_parameters(nh_private);

    fc_status_ = 0;
    vc_status_ = 0;
    //TODO: Inkyu, these parameters should be read from a config file
    //obtained from nonlinear optimization.
    rollScale_ = 0.000865;
    pitchScale_ = 0.000844;

    yawrateScale_ = 0.002241;
    heightScale_ = 0.002649;
    rollDeadZone_ = 19.8;
    pitchDeadZone_ = 19.8;
    heightDeadZone_ = 39.6;
    vcNeutral_ = 1024;
    rollTrim_ = 62.757;
    pitchTrim_ = -39.983;

    int groundstation_enable;
    nh_private.param("groundstation_enable", groundstation_enable, 1);
    if (groundstation_enable)
    {
        dji_sdk_mission = new DJISDKMission(nh);
    }
}

void DJISDKNode::gps_convert_ned(float &ned_x, float &ned_y,
                                 double gps_t_lon, double gps_t_lat,
                                 double gps_r_lon, double gps_r_lat)
{
    double d_lon = gps_t_lon - gps_r_lon;
    double d_lat = gps_t_lat - gps_r_lat;
    ned_x = DEG2RAD(d_lat) * C_EARTH;
    ned_y = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
};

dji_sdk::LocalPosition DJISDKNode::gps_convert_ned(dji_sdk::GlobalPosition loc)
{
    dji_sdk::LocalPosition local;
    gps_convert_ned(local.x, local.y,
                    loc.longitude, loc.latitude,
                    global_position_ref.longitude, global_position_ref.latitude);
    local.z = loc.height;
    return local;
}

void DJISDKNode::external_transform_subscriber_callback(geometry_msgs::TransformStamped transform)
{
    external_position.x = transform.transform.translation.x;
    external_position.y = transform.transform.translation.y;
    external_position.z = transform.transform.translation.z;
    external_position_publisher.publish(external_position);
}

void DJISDKNode::cmdCallBack(const mav_msgs::RollPitchYawrateThrustConstPtr &msg)
{
    //std::cout<<"cmdCallBack"<<std::endl;
    if (fc_status_ && !vc_status_)
    {
        DJI::onboardSDK::VirtualRCSetting vrc_setting;
        vrc_setting.enable = 1;
        vrc_setting.cutoff = 1;
        rosAdapter->virtualRC->setControl((bool)vrc_setting.enable, (DJI::onboardSDK::VirtualRC::CutOff)vrc_setting.cutoff);
        vc_status_ = 1;
    }
    if (fc_status_ && vc_status_)
    {
        //cout<<"msg->roll="<<msg->roll<<" msg->pitch="<<msg->pitch<<" msg->yaw_rate="<<msg->yaw_rate<<" msg->thrust="<<msg->thrust.z<<endl;
        //Rolling
        virtual_rc_data[0] = 1024;
        virtual_rc_data[1] = 1024;
        virtual_rc_data[2] = 1024;
        virtual_rc_data[3] = 1024;
        virtual_rc_data[4] = 1324;
        //virtual_rc_data[6] = 1552;
        virtual_rc_data[6] = 1024; //A mode

        //The below scaling factors are obtained from non-linear regression btw
        //commands input and output measurements. The measurements can be obatined either
        //DJI simulator or Vicon.
        //More details can be found from the provided documentation.

        //Max angle of attack = 30deg
        virtual_rc_data[0] = (uint32_t)((msg->roll / rollScale_) + vcNeutral_ + rollTrim_);    //roll angle cmd
        virtual_rc_data[1] = (uint32_t)((msg->pitch / pitchScale_) + vcNeutral_ + pitchTrim_); //pitch angle cmd
        virtual_rc_data[2] = (uint32_t)((msg->thrust.z / heightScale_) + vcNeutral_);          //vertical vel cmd
        virtual_rc_data[3] = (uint32_t)((-msg->yaw_rate / yawrateScale_) + vcNeutral_);        //yaw rate cmd
        deadZoneRecovery(virtual_rc_data);
        auto cur_time = ros::Time::now();
        vcCommand_.header.stamp = cur_time;
        vcCommand_.roll = double(virtual_rc_data[0]);
        vcCommand_.pitch = double(virtual_rc_data[1]);
        vcCommand_.yaw_rate = double(virtual_rc_data[3]);
        vcCommand_.thrust.z = double(virtual_rc_data[2]);

        //drone_->virtual_rc_control(virtual_rc_data);
        vrc_.roll = virtual_rc_data[0];
        vrc_.pitch = virtual_rc_data[1];
        vrc_.throttle = virtual_rc_data[2];
        vrc_.yaw = virtual_rc_data[3];
        vrc_.gear = virtual_rc_data[4];
        vrc_.reserved = virtual_rc_data[5];
        vrc_.mode = virtual_rc_data[6];
        vrc_.Channel_07 = virtual_rc_data[7];
        vrc_.Channel_08 = virtual_rc_data[8];
        vrc_.Channel_09 = virtual_rc_data[9];
        vrc_.Channel_10 = virtual_rc_data[10];
        vrc_.Channel_11 = virtual_rc_data[11];
        vrc_.Channel_12 = virtual_rc_data[12];
        vrc_.Channel_13 = virtual_rc_data[13];
        vrc_.Channel_14 = virtual_rc_data[14];
        vrc_.Channel_15 = virtual_rc_data[15];
        rosAdapter->virtualRC->sendData(vrc_);
        vc_cmd_pub_.publish(vcCommand_);
    }
}

void DJISDKNode::deadZoneRecovery(uint32_t *pVC_cmd)
{
    if (*pVC_cmd <= (uint32_t)(vcNeutral_ + rollDeadZone_) && *pVC_cmd >= vcNeutral_) // roll dead-zone
    {
        *pVC_cmd = (uint32_t)(vcNeutral_ + rollDeadZone_);
        //printf("roll pos deadzone detected\n");
    }
    else if (*pVC_cmd >= (uint32_t)(vcNeutral_ - rollDeadZone_) && *pVC_cmd < vcNeutral_)
    {
        *pVC_cmd = (uint32_t)(vcNeutral_ - rollDeadZone_);
    }

    if (*(pVC_cmd + 1) <= (uint32_t)(vcNeutral_ + pitchDeadZone_) && *(pVC_cmd + 1) >= vcNeutral_) // pitch dead-zone
    {
        *(pVC_cmd + 1) = (uint32_t)(vcNeutral_ + pitchDeadZone_);
    }
    else if (*(pVC_cmd + 1) >= (uint32_t)(vcNeutral_ - pitchDeadZone_) && *(pVC_cmd + 1) < vcNeutral_)
    {
        *(pVC_cmd + 1) = (uint32_t)(vcNeutral_ - pitchDeadZone_);
    }
#if 0
  if (*(pVC_cmd+2)<=(uint32_t)(vcNeutral_+heightDeadZone_) && *(pVC_cmd+2)>=vcNeutral_) // height dead-zone
  {
    *(pVC_cmd+2)=(uint32_t)(*(pVC_cmd+2)+heightDeadZone_);
  }
  else if(*(pVC_cmd+2)>=(uint32_t)(vcNeutral_-heightDeadZone_) && *(pVC_cmd+2)<vcNeutral_)
  {
    *(pVC_cmd+2)=(uint32_t)(*(pVC_cmd+2)-heightDeadZone_);
  }
#endif
}

void DJISDKNode::keyboardCallBack(const keyboard::KeyConstPtr &msg)
{
    ROS_INFO("keyboard callback");
    switch (msg->code)
    {
    case 'b':
        rosAdapter->coreAPI->setControl(1);
        cout << "Success in obtaining control!!" << endl;
        fc_status_ = 1;
        break;

    case 'c':
        cout << "Releasing SDK control permit" << endl;
        rosAdapter->coreAPI->setControl(0);
        fc_status_ = 0;
        vc_status_ = 0;
        break;
    }
}

#if 0
void DJISDKNode::joyCallBack(const sensor_msgs::Joy::ConstPtr& msg)
{
	ROS_INFO("JoyCallBack");
}
#endif
