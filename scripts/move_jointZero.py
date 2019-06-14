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


joint_command = [0,0,0,0,0,0,0]



rospy.init_node('move_test_1', anonymous=True)

# moveit
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander('manipulator')

util.speed_set(group,0.01)
plan = util.fK_point_calculate(group,JointAngle=joint_command)
util.execute_plan(group,plan)    
# ipdb.set_trace()
moveit_commander.roscpp_shutdown()
# Exit MoveIt
moveit_commander.os._exit(0)