#!/usr/bin/env python
"""
Mark robot's position
refer to https://www.programcreek.com/python/example/88812/visualization_msgs.msg.Marker
"""

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped,Point

import sys
if len(sys.argv) < 2 or not sys.argv[1] == "true":
    exit(0)

print("Running with path visualization")

def callback(data):
    add_point = Point()
    add_point.x = data.pose.pose.position.x
    add_point.y = data.pose.pose.position.y
    add_point.z = 0
    marker.points.append(add_point)
    # Publish the Marker
    pub_point.publish(marker)


marker = Marker()
marker.header.frame_id = "map"
marker.type = marker.LINE_STRIP
marker.action = marker.ADD

# marker scale
marker.scale.x = 0.03
marker.scale.y = 0.03
marker.scale.z = 0.03

# marker color
marker.color.a = 1.0
marker.color.r = 0.0
marker.color.g = 0.0
marker.color.b = 1.0

# marker orientaiton
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0

# marker line points
marker.points = []
rospy.loginfo('Marker created')

rospy.init_node('path_tracker')

pub_point = rospy.Publisher('tracked_path', Marker, queue_size=1)
print ("Publisher created....")

rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped, callback)
print("Subcriber created....")
rospy.spin()
