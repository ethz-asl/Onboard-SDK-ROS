#!/usr/bin/env python
# Node to create a Polygon / GPS path file from points clicked on the RVIZ gui

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from math import *
import yaml
import rospy
import random

class RVIZPathCreator:

    def __init__(self):
        self.marker_pub = rospy.Publisher('clicked_points_markers', MarkerArray, queue_size=10)
        self.marker_array = MarkerArray()
        self.clicked_point_sub = rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback, tcp_nodelay=True)
        # Ask the user for number of polygon vertices
        self.num_polygon_vertices = input("Enter Number of Polygon Vertices: ")
        self.clicked_points = 0
        self.yaml_dict = {'polygon': {'hull':[]}}
        
    def clicked_point_callback(self, data):
        self.yaml_dict['polygon']['hull'].append([data.point.x,data.point.y])
        marker = self.init_marker()
        marker.pose.position = data.point
        self.marker_array.markers.append(marker)
        self.marker_pub.publish(self.marker_array)
        print("Added point")
        self.clicked_points += 1
        if (self.clicked_points == self.num_polygon_vertices):
            self.write_yaml()
            rospy.signal_shutdown("All vertices clicked")
    
    def init_marker(self):
        marker = Marker()
        marker.header.frame_id = '/odom'
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.id = self.clicked_points
        marker.lifetime = rospy.Duration.from_sec(0.0)
        marker.pose.orientation.w = 1.0

        return marker

    def write_yaml(self):
        with open('data.yml', 'w') as outfile:
            yaml.dump(self.yaml_dict, outfile, default_flow_style=True)

if __name__ == '__main__':

  try:
    rospy.init_node('rviz_path_creator', anonymous=True)
    path_creator = RVIZPathCreator()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
