#!/usr/bin/env python
# Node to spoof an external transform for DJI waypoint navigation service from local position received from simulator

from geometry_msgs.msg import TransformStamped
import rospy
import dji_sdk.msg

class ExternalTransformSpoofer:

    def __init__(self):
        self.got_local_position = False
        self.local_position_sub = rospy.Subscriber('local_position', dji_sdk.msg.LocalPosition, self.local_position_callback, tcp_nodelay=True)
        self.spoofed_external_transform_pub = rospy.Publisher('spoofed_external_transform', TransformStamped, queue_size=1)

    def local_position_callback(self, data):
        if (~self.got_local_position):
            self.got_local_position = True
        spoofed_external_transform = TransformStamped()
        spoofed_external_transform.header = data.header
        spoofed_external_transform.child_frame_id = 'spoofed_external_transform'
        spoofed_external_transform.transform.translation.x = data.x + 5.0
        spoofed_external_transform.transform.translation.y = data.y + 5.0
        spoofed_external_transform.transform.translation.z = data.z
        spoofed_external_transform.transform.rotation.x = 1.0
        spoofed_external_transform.transform.rotation.y = 0.0
        spoofed_external_transform.transform.rotation.z = 0.0
        spoofed_external_transform.transform.rotation.w = 0.0
        self.spoofed_external_transform_pub.publish(spoofed_external_transform)

if __name__ == '__main__':

  try:
    rospy.init_node('external_transform_spoofer', anonymous=True)
    gs = ExternalTransformSpoofer()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
