#!/usr/bin/env python
# Node to convert DJI-specific messages into normal ROS messages

import dji_sdk.msg
import rospy
import tf
from geometry_msgs.msg import PointStamped, QuaternionStamped

class Dji2RosWrapper:

    def __init__(self):
        self.transform_broadcaster = tf.TransformBroadcaster()
        self.local_position = PointStamped()
        self.quaternion = QuaternionStamped()

        self.local_position_sub = rospy.Subscriber('dji_sdk/local_position', dji_sdk.msg.LocalPosition, self.local_position_callback, tcp_nodelay=True)
        self.attitude_quaternion_sub = rospy.Subscriber('dji_sdk/attitude_quaternion', dji_sdk.msg.AttitudeQuaternion, self.attitude_quaternion_callback, tcp_nodelay=True)
        self.local_position_pub = rospy.Publisher('ros/local_position', PointStamped, queue_size=1)

    def local_position_callback(self, data):
        self.local_position.header = data.header
        self.local_position.point.x = data.x
        self.local_position.point.y = data.y
        self.local_position.point.z = data.z
        self.quaternion.header = data.header
        self.publish_ros_messages()

    def attitude_quaternion_callback(self, data):
        self.quaternion.header = data.header
        self.quaternion.quaternion.x = data.q2
        self.quaternion.quaternion.y = data.q3
        self.quaternion.quaternion.z = data.q1
        self.quaternion.quaternion.w = data.q0
        self.publish_ros_messages()

    def publish_ros_messages(self):
        self.transform_broadcaster.sendTransform((self.local_position.point.x, self.local_position.point.y, self.local_position.point.z),
                        (self.quaternion.quaternion.x, self.quaternion.quaternion.y, self.quaternion.quaternion.z, self.quaternion.quaternion.w),
                        rospy.Time.now(),
                        "dji_matrice",
                        "world")
        self.local_position_pub.publish(self.local_position)

if __name__ == '__main__':

  try:
    rospy.init_node('external_transform_spoofer', anonymous=True)
    gs = Dji2RosWrapper()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
