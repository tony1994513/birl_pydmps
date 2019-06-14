#!/usr/bin/env python
import numpy as np
import pandas as pd
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,Pose
import ipdb
from scipy.ndimage.filters import gaussian_filter1d
from birl_pydmps import util
import random

def main():
    rospy.init_node('test', anonymous=True)
    raw_data = np.load('record_demo.npy')
    # ipdb.set_trace()
    filtered_data = gaussian_filter1d(raw_data.T, sigma=5).T
    # filtered_data = util.filter_static_points(filtered_data,0.03)
    marker_pub = rospy.Publisher("/iiwa/visualization_marker", Marker, queue_size=100)
    rospy.sleep(0.5)
    rgba_tuple = [random.uniform(0, 1), random.uniform(0, 1), random.uniform(0.5, 1), 1]
    for idx, point in enumerate(filtered_data):
        # ipdb.set_trace()
        now_pose = Pose()
        now_pose.position.x = point[0]
        now_pose.position.y = point[1]
        now_pose.position.z = point[2]  
        util.send_traj_point_marker(marker_pub=marker_pub, pose=now_pose, id=idx, rgba_tuple=rgba_tuple)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass