#!/usr/bin/env python
import numpy as np
import pandas as pd
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from scipy.ndimage.filters import gaussian_filter1d
from birl_pydmps import DMPs_rhythmic
from birl_pydmps import DMPs_discrete
import ipdb
from birl_pydmps import util
from visualization_msgs.msg import Marker
import random


rospy.init_node('robot_move_test', anonymous=True)
raw_data = np.load('record_demo.npy')
filtered_data = gaussian_filter1d(raw_data.T, sigma=5).T

dmp = DMPs_rhythmic(n_dmps=filtered_data.shape[1], n_bfs=200)
dmp.imitate_path(y_des=filtered_data.T)
y_track, dy_track, ddy_track = dmp.rollout()

# dmp.y0[2] = dmp.y0[2] 
# dmp.goal[2] = dmp.goal[2] - 0.1
# dmp.goal[1] = dmp.goal[1]+ 0.1

marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)
rospy.sleep(0.5)
rgba_tuple = [random.uniform(0, 1), random.uniform(0, 1), random.uniform(0.5, 1), 1]
for idx, point in enumerate(y_track):
    now_pose = Pose()
    now_pose.position.x = point[0]
    now_pose.position.y = point[1]
    now_pose.position.z = point[2]  
    util.send_traj_point_marker(marker_pub=marker_pub, pose=now_pose, id=idx, rgba_tuple=rgba_tuple)

# moveit
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander('manipulator')
reference_frame = 'world'
group.set_pose_reference_frame(reference_frame)

plan = util.ik_cartesain_path(group, y_track)
util.execute_plan(group,plan)
# print "demonstration"
# print raw_data[-1]
# print "DMP"
# print group.get_current_pose().pose
moveit_commander.roscpp_shutdown()
# Exit MoveIt
moveit_commander.os._exit(0)



