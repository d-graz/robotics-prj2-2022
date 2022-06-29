#!/bin/python

import sys
import rospy
from costmap_processing.srv import SavePath

import os
cwd = os.getcwd()

def askSavePath(filename):
    rospy.wait_for_service('save_path')
    try:
        save_path = rospy.ServiceProxy('save_path', SavePath)

        if not filename[0] == '/':
            # it's not an absolute path
            filename = cwd + '/' + filename

        resp = save_path(filename)
        print(resp.str)
    except rospy.ServiceException as e:
        print("Failed to contact service: %s"%e)

if __name__ == "__main__":
    if len(sys.argv) == 2:
        askSavePath(sys.argv[1])
    else:
        print("Please specify the name of the output file")
        sys.exit(1)