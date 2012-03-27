#!/usr/bin/env python
'''
Created on Mar 26, 2012

Adapted from: http://www.ros.org/wiki/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action

@author: ab2283
'''
import roslib; roslib.load_manifest('arm_controller')
import rospy
import actionlib
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib_msgs.msg._GoalStatus import GoalStatus
 
def get_trajectory_point(positions, time_from_start):
    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = [.0] * len(positions)
    point.time_from_start = time_from_start
    return point

def task_completed(status, result):
    print "Succeeded" if status == GoalStatus.SUCCEEDED else "Failed"

def move_right_arm_initial_pose():
    client = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_action', JointTrajectoryAction)
    client.wait_for_server()
    
    goal = JointTrajectoryGoal()
    goal.trajectory.joint_names = ['r_shoulder_pan_joint', # min= -2.1, max = 0.56
                                   'r_shoulder_lift_joint', # min = -0.35, max = 1.28
                                   'r_upper_arm_roll_joint', # min = -3.75, max = 0.65
                                   'r_elbow_flex_joint', # min = -2.12, max = -0.15
                                   'r_forearm_roll_joint', # rotation joints need different logic
                                   'r_wrist_flex_joint', # min = -2, max = -0.1
                                   'r_wrist_roll_joint'] # rotation joints need different logic
    point = get_trajectory_point([-2, -0.35 , -1, -1.8, 0, 0, 0.0], roslib.rostime.Duration(0))
    goal.trajectory.points.append(point)
    
    goal.trajectory.header.stamp = rospy.Time.now()
    client.send_goal(goal, done_cb=task_completed)
    print "Right Arm Goal Sent"
    client.wait_for_result()
    print "Right Arm Initialized."
    

def move_left_arm_initial_pose():
    client = actionlib.SimpleActionClient('l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
    client.wait_for_server()
    
    goal = JointTrajectoryGoal()
    goal.trajectory.joint_names = ['l_shoulder_pan_joint', # min= -0.56, max = 2.1, opposite of right
                                   'l_shoulder_lift_joint', # min = -0.35, max = 1.28, same as right
                                   'l_upper_arm_roll_joint', # min = -0.65, max = 3.75, opposite of right
                                   'l_elbow_flex_joint', # min = -2.12, max = -0.15, same as right
                                   'l_forearm_roll_joint', # rotation joints need different logic
                                   'l_wrist_flex_joint', # min = -2, max = -0.1
                                   'l_wrist_roll_joint'] # rotation joints need different logic
    point = get_trajectory_point([2, -0.35, 1, -1.8, 0, 0, 0.0], roslib.rostime.Duration(0))
    goal.trajectory.points.append(point)
    
    goal.trajectory.header.stamp = rospy.Time.now()
    client.send_goal(goal, done_cb=task_completed)
    print "Left Arm Goal Sent"
    client.wait_for_result()
    print "Left Arm Initialized."
    
def move_head_initial_pose():
    client = actionlib.SimpleActionClient('head_traj_controller/joint_trajectory_action', JointTrajectoryAction)
    client.wait_for_server()
    
    goal = JointTrajectoryGoal()
    goal.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
    point = get_trajectory_point([-0.001, 0.714], roslib.rostime.Duration(0))
    goal.trajectory.points.append(point)
    
    goal.trajectory.header.stamp = rospy.Time.now()
    client.send_goal(goal, done_cb=task_completed)
    print "Head Goal Sent"
    client.wait_for_result()
    print "Head Initialized."
      
if __name__ == '__main__':
    rospy.init_node('move_arm_trajectory_client')
    move_right_arm_initial_pose()
    move_left_arm_initial_pose()
    move_head_initial_pose()
