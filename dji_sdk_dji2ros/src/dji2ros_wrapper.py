#!/usr/bin/env python
# Node to convert DJI-specific messages into normal ROS messages

import dji_sdk.msg
import rospy
import tf
from geometry_msgs.msg import PointStamped, QuaternionStamped, PoseStamped, Point

class Dji2RosWrapper:

    def __init__(self):
        self.transform_broadcaster = tf.TransformBroadcaster()
        self.local_position = PointStamped()
        self.external_position = PointStamped()
        self.quaternion = QuaternionStamped()
        self.local_pose = PoseStamped()
        self.external_pose = PoseStamped()

        self.external_position_sub = rospy.Subscriber('dji_sdk/external_position', Point, self.external_position_callback, tcp_nodelay=True)
        self.local_position_sub = rospy.Subscriber('dji_sdk/local_position', dji_sdk.msg.LocalPosition, self.local_position_callback, tcp_nodelay=True)
        self.attitude_quaternion_sub = rospy.Subscriber('dji_sdk/attitude_quaternion', dji_sdk.msg.AttitudeQuaternion, self.attitude_quaternion_callback, tcp_nodelay=True)

        self.local_position_pub = rospy.Publisher('ros/local_position', PointStamped, queue_size=1)
        self.external_position_pub = rospy.Publisher('ros/external_position', PointStamped, queue_size=1)
        self.local_pose_pub = rospy.Publisher('ros/local_pose', PoseStamped, queue_size=1)
        self.external_pose_pub = rospy.Publisher('ros/external_pose', PoseStamped, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.publish_callback)

    def external_position_callback(self, data):
        self.external_position.header.stamp = rospy.Time.now()
        self.external_position.header.frame_id = "/world"

        self.external_position.point.x = data.x
        self.external_position.point.y = data.y
        self.external_position.point.z = data.z
        self.external_pose.pose.position = self.external_position.point

    def local_position_callback(self, data):
        self.local_position.header = data.header
        self.local_position.point.x = data.x
        self.local_position.point.y = data.y
        self.local_position.point.z = data.z
        self.local_pose.header = data.header
        self.local_pose.pose.position = self.local_position.point

    def attitude_quaternion_callback(self, data):
        self.quaternion.header = data.header
        self.quaternion.quaternion.x = data.q2
        self.quaternion.quaternion.y = data.q3
        self.quaternion.quaternion.z = data.q1
        self.quaternion.quaternion.w = data.q0
        self.local_pose.header = data.header
        self.local_pose.pose.orientation = self.quaternion.quaternion
        self.external_pose.header = data.header
        self.external_pose.pose.orientation = self.quaternion.quaternion

    def publish_callback(self, event):
        self.transform_broadcaster.sendTransform((self.local_position.point.x, self.local_position.point.y, self.local_position.point.z),
                        (self.quaternion.quaternion.x, self.quaternion.quaternion.y, self.quaternion.quaternion.z, self.quaternion.quaternion.w),
                        rospy.Time.now(),
                        "dji_matrice",
                        "world")
        self.local_position_pub.publish(self.local_position)
        self.local_pose_pub.publish(self.local_pose)
        self.external_position_pub.publish(self.external_position)
        self.external_pose_pub.publish(self.external_pose)

if __name__ == '__main__':

  try:
    rospy.init_node('external_transform_spoofer', anonymous=True)
    gs = Dji2RosWrapper()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
