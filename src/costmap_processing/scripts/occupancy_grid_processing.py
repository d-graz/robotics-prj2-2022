#!/bin/python

# path color
path_red = 0
path_green = 0
path_blue = 255
path_thickness = 1
scale = 0.05

# Importing required packages
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import os

# Get current working directory
cwd = os.getcwd()

image = False
origin_set = False
base_pose = (0,0)
prev_position = (0,0)
magnify = 1/scale
filename = ""

def callback(data):

    origin = data.info.origin.position

    global origin_set
    global image

    # Initialize map
    width = data.info.width
    height = data.info.height
    size =  (width , height)
    image = np.zeros(size)
    
    counter = 0
    #print (len(data.data))
    for p in data.data:
      image[counter%width][int(counter/width)] = p
      counter+=1
    
    # do magic things with OpenCV
    kernel = np.ones((2,2),np.uint8)
    image = cv2.dilate(image,kernel,iterations = 1)
    image = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    custom_map = OccupancyGrid()

    custom_map.header = data.header
    custom_map.info = data.info
    custom_map.data =  np.transpose(image).flatten().astype (int)

    image = cv2.rotate(image, cv2.ROTATE_180)
    image[image == -1] = 150
    image[image == 0] = 255
    image[image == 100] = 0

    global base_pose
    global prev_position

    if not origin_set:
        base_pose = (int(height + origin.y*magnify), int(width + origin.x*magnify))
        print("Setting robot's origin position to", base_pose)
        origin_set = True
    prev_position = base_pose
    
    print("saving map", filename)
    cv2.imwrite (filename,image)

color = (path_red, path_green, path_blue)

def pathCallback(data):

    tmp_x = data.pose.pose.position.x
    tmp_y = data.pose.pose.position.y

    new_position = (int(base_pose[0]-tmp_y*magnify), int(base_pose[1]-tmp_x*magnify))

    global prev_position

    cv2.line(image, prev_position, new_position, color, path_thickness)
    prev_position = new_position
    print("drawing step [", prev_position, new_position, "]")
    cv2.imwrite (filename,image)

def listener():

    import sys
    if len(sys.argv) < 2:
        rospy.logerr("Please specify name of the map to save 'path/filename.png'")
        exit(1)
    
    global filename
    filename = sys.argv[1]

    # node name
    rospy.init_node('map_tracker', anonymous=False)

    # respond to map update
    rospy.Subscriber("/map", OccupancyGrid, callback)

    # respond to robot movement
    rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped, pathCallback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
