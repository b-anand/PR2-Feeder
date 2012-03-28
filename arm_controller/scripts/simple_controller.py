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
from ee_cart_imped_action import EECartImpedClient
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest

def get_trajectory_point(positions, time_from_start):
    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = [.0] * len(positions)
    point.time_from_start = time_from_start
    return point

def task_completed(status, result):
    print "Succeeded" if status == GoalStatus.SUCCEEDED else "Failed"

def move_body_part(service, joints, positions):
    client = actionlib.SimpleActionClient(service, JointTrajectoryAction)
    client.wait_for_server()
    
    goal = JointTrajectoryGoal()
    goal.trajectory.joint_names = joints
    point = get_trajectory_point(positions, roslib.rostime.Duration(0))
    goal.trajectory.points.append(point)
    
    goal.trajectory.header.stamp = rospy.Time.now()
    client.send_goal(goal, done_cb=task_completed)
    print "Goal Sent"
    client.wait_for_result()
    print "Success."


def move_right_arm(positions):
    service = 'r_arm_controller/joint_trajectory_action'
    
    joints = [ 'r_shoulder_pan_joint', # min(left) = -2.1, max(right) = 0.56
               'r_shoulder_lift_joint', # min(up) = -0.35, max(down) = 1.28
               'r_upper_arm_roll_joint', # min = -3.75, max = 0.65
               'r_elbow_flex_joint', # min = -2.12, max = -0.15
               'r_forearm_roll_joint', # rotation joints need different logic
               'r_wrist_flex_joint', # min = -2, max = -0.1
               'r_wrist_roll_joint'] # rotation joints need different logic
    
    move_body_part(service, joints, positions)

def move_left_arm(positions):
    service = 'l_arm_controller/joint_trajectory_action'
    
    joints = [ 'l_shoulder_pan_joint', # min= -0.56, max = 2.1, opposite of right
               'l_shoulder_lift_joint', # min = -0.35, max = 1.28, same as right
               'l_upper_arm_roll_joint', # min = -0.65, max = 3.75, opposite of right
               'l_elbow_flex_joint', # min = -2.12, max = -0.15, same as right
               'l_forearm_roll_joint', # rotation joints need different logic
               'l_wrist_flex_joint', # min = -2, max = -0.1
               'l_wrist_roll_joint'] # rotation joints need different logic
    
    move_body_part(service, joints, positions)

def move_head(positions):
    service = 'head_traj_controller/joint_trajectory_action'
    joints = ['head_pan_joint', 'head_tilt_joint']
    move_body_part(service, joints, positions)

def move_right_arm_initial_pose():
    positions = [-2, -0.35 , -1, -1.8, 0, 0, 0.0]
    move_right_arm(positions)

def move_left_arm_initial_pose():
    positions = [2, -0.35, 1, -1.8, 0, 0, 0.0]
    move_left_arm(positions)

def move_head_initial_pose():
    positions = [-0.001, 0.714]
    move_head(positions)

def imped_right_arm_control():
    control = EECartImpedClient('right_arm')
    control.addTrajectoryPoint(0.75, 0, 0, 0, 0, 0, 1,
                               10, 1000, 1000, 30, 30, 30,
                               False, False, False, False, False, False,
                               1, '/torso_lift_link');
    control.addTrajectoryPoint(0.6, 0, 0, 0, 0, 0, 1,
                               100, 1000, 1000, 30, 30, 30,
                               True, False, False, False, False, False, 
                               2, '/torso_lift_link');
    control.sendGoal()
    
def detect_objects():
    service_name = '/object_detection'
    rospy.wait_for_service(service_name)
    try:
        table_object_detector = rospy.ServiceProxy(service_name, TabletopDetection)
        
        req = TabletopDetectionRequest()
        req.return_clusters = False
        req.return_models = True
        req.num_models = 1
        
        resp = table_object_detector(req)
        if resp.detection.result == resp.detection.SUCCESS:
            point = resp.detection.clusters[0].points[0]
            print point
        else:
            print resp.detection.result
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    rospy.init_node('move_arm_trajectory_client')
    joints = [ 'r_shoulder_pan_joint', # min(left) = -2.1, max(right) = 0.56
               'r_shoulder_lift_joint', # min(up) = -0.35, max(down) = 1.28
               'r_upper_arm_roll_joint', # min(clockwise_for_robot) = -3.75, max(anticlockwise) = 0.65
               'r_elbow_flex_joint', # min = -2.12, max = -0.15
               'r_forearm_roll_joint', # rotation joints need different logic
               'r_wrist_flex_joint', # min = -2, max = -0.1
               'r_wrist_roll_joint'] # rotation joints need different logic
    
    '''
    positions = [2.0] + [0.0] * 6
    move_left_arm(positions)
    '''
    
    positions = [-0.5, -0.18, -2, -1, 0, 0, 0]
    move_right_arm(positions)
    
    '''
    move_right_arm_initial_pose()
    move_left_arm_initial_pose()
    move_head_initial_pose()
    detect_objects()
    '''
    
    
