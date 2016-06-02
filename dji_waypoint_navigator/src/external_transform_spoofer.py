#!/usr/bin/env python
# Node to spoof an external transform for DJI waypoint navigation service from local position received from simulator

from geometry_msgs.msg import TransformStamped
from math import *
import rospy
import random
import dji_sdk.msg

class ExternalTransformSpoofer:

    def __init__(self):
        self.got_local_position = False
        self.spoofed_external_transform = TransformStamped()
        self.spoofed_external_transform.child_frame_id = 'spoofed_external_transform'

        self.local_position_sub = rospy.Subscriber('local_position', dji_sdk.msg.LocalPosition, self.local_position_callback, tcp_nodelay=True)
        self.spoofed_external_transform_pub = rospy.Publisher('spoofed_external_transform', TransformStamped, queue_size=1)

        # Maximum noise radius (polar coordinates) [m]
        self.max_noise_radius = rospy.get_param('~max_noise_radius', 0.0)
        self.R_noise = 0.0
        self.theta_noise = 0.0

    def local_position_callback(self, data):
        if (~self.got_local_position):
            self.got_local_position = True
        self.spoofed_external_transform.header = data.header
        self.spoofed_external_transform.transform.translation.x = data.x + 5.0
        self.spoofed_external_transform.transform.translation.y = data.y + 5.0
        self.spoofed_external_transform.transform.translation.z = data.z
        self.spoofed_external_transform.transform.rotation.x = 1.0
        self.spoofed_external_transform.transform.rotation.y = 0.0
        self.spoofed_external_transform.transform.rotation.z = 0.0
        self.spoofed_external_transform.transform.rotation.w = 0.0
        self.sample_noise()
        self.spoofed_external_transform_pub.publish(self.spoofed_external_transform)

    def sample_noise(self):
        # Generate noise in x and y
        self.R_noise = random.uniform(0, self.max_noise_radius)
        self.theta_noise = random.uniform(0, 2*pi)
        # Add noise to spoofed output
        self.spoofed_external_transform.transform.translation.x += self.R_noise*cos(self.theta_noise)
        self.spoofed_external_transform.transform.translation.y += self.R_noise*sin(self.theta_noise)

if __name__ == '__main__':

  try:
    rospy.init_node('external_transform_spoofer', anonymous=True)
    gs = ExternalTransformSpoofer()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
