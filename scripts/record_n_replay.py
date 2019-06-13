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

def main():
    rospy.init_node('cartesian_move_test', anonymous=True)
    raw_data = np.load('record_demo.npy')
    filtered_data = gaussian_filter1d(raw_data.T, sigma=5).T
    # util.plot_3d_demo(filtered_data)

    # moveit
    moveit_commander.roscpp_initialize(sys.argv)
    
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('manipulator')

    # reference_frame = 'iiwa_link_ee'
    # group.set_pose_reference_frame(reference_frame)
    
    joint_command = [0.11133063584566116, 0.5712737441062927,
    -0.09774867445230484, -1.7133415937423706, -0.036780450493097305, -0.5433750152587891, 0.17699939012527466]

    util.speed_set(group,0.01)
    plan = util.fK_point_calculate(group,JointAngle=joint_command)
    util.execute_plan(group,plan) 

    plan = util.iK_point_calculate(group,point=filtered_data[0])
    util.execute_plan(group,plan)    

    plan = util.ik_cartesain_path(group, filtered_data)
    util.execute_plan(group,plan)
    
    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()
    # Exit MoveIt
    moveit_commander.os._exit(0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass