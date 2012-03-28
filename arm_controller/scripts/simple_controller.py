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
from arm_navigation_msgs.msg import MoveArmAction, MoveArmActionGoal, MoveArmGoal, JointConstraint
from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse, GetKinematicSolverInfo, GetKinematicSolverInfoRequest


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
    
    joints = [ 'r_shoulder_pan_joint', # min(right_for_robot) = -2.1, max(left) = 0.56
               'r_shoulder_lift_joint', # min(up) = -0.35, max(down) = 1.28
               'r_upper_arm_roll_joint', # min(clockwise_for_robot) = -3.75, max(anticlockwise) = 0.65
               'r_elbow_flex_joint', # min(near/up) = -2.12, max(away/down) = -0.15
               'r_forearm_roll_joint', # rotation joints need different logic, 5 = hand camera is down
               'r_wrist_flex_joint', # min(folded) = -2, max(open) = -0.1
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

def move_right_arm_pickup_start_state():
    positions = [-0.5, -0.18, -2, -1.2, 5, -1.1, 0]
    move_right_arm(positions)

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

def move_arm_completed_cb(status, result):
    print "Succeeded" if status == GoalStatus.SUCCEEDED else "Failed"
    print type(result)
    print result

def move_arm_ik():
    service = "move_right_arm"
    client = actionlib.SimpleActionClient(service, MoveArmAction)
    client.wait_for_server()
    
    goal = MoveArmGoal()
    goal.planner_service_name = "ompl_planning/plan_kinematic_path"
    goal.motion_plan_request.group_name = "right_arm"
    goal.motion_plan_request.num_planning_attempts = 1
    goal.motion_plan_request.allowed_planning_time = roslib.rostime.Duration(5)
    goal.motion_plan_request.planner_id = ""

    joint_names = [  "r_shoulder_pan_joint",
                     "r_shoulder_lift_joint",
                     "r_upper_arm_roll_joint",
                     "r_elbow_flex_joint",
                     "r_forearm_roll_joint",
                     "r_wrist_flex_joint",
                     "r_wrist_roll_joint"]
    
    joint_constraints = []
    for joint_name in joint_names:
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = 0
        constraint.tolerance_above = 0.1
        constraint.tolerance_below = 0.1
        joint_constraints.append(constraint)
    
    joint_constraints[0].position = -2
    joint_constraints[3].position = -0.2
    joint_constraints[5].position = -0.15
    
    goal.motion_plan_request.goal_constraints.joint_constraints = joint_constraints
    client.send_goal(goal, done_cb=move_arm_completed_cb)
    client.wait_for_result()
    
    
def get_ik_info():
    service_name = "pr2_right_arm_kinematics/get_ik_solver_info"
    rospy.wait_for_service(service_name)
    try:
        ik_solver_info = rospy.ServiceProxy(service_name, GetKinematicSolverInfo)
        req = GetKinematicSolverInfoRequest()
        resp = ik_solver_info(req)
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def solve_ik():
    joint_names = [  "r_shoulder_pan_joint",
                     "r_shoulder_lift_joint",
                     "r_upper_arm_roll_joint",
                     "r_elbow_flex_joint",
                     "r_forearm_roll_joint",
                     "r_wrist_flex_joint",
                     "r_wrist_roll_joint"]
    
    service_name = "pr2_right_arm_kinematics/get_ik"
    rospy.wait_for_service(service_name)
    try:
        ik_solver = rospy.ServiceProxy(service_name, GetPositionIK)
        req = GetPositionIKRequest()
        req.timeout = roslib.rostime.Duration(5)
        req.ik_request.ik_link_name = "r_wrist_roll_link"
        req.ik_request.pose_stamped.header.frame_id = "torso_lift_link"

        req.ik_request.pose_stamped.pose.position.x = 0.75;
        req.ik_request.pose_stamped.pose.position.y = -0.188;
        req.ik_request.pose_stamped.pose.position.z = 0.0;

        req.ik_request.pose_stamped.pose.orientation.x = 0.0;
        req.ik_request.pose_stamped.pose.orientation.y = 0.0;
        req.ik_request.pose_stamped.pose.orientation.z = 0.0;
        req.ik_request.pose_stamped.pose.orientation.w = 1.0;
        
        req.ik_request.ik_seed_state.joint_state.position = [0] * 7
        req.ik_request.ik_seed_state.joint_state.name = joint_names
        
        resp = GetPositionIKResponse()
        resp = ik_solver(req)
        
        print resp.solution.joint_state.name
        print resp.solution.joint_state.position
        
        move_right_arm(resp.solution.joint_state.position)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    rospy.init_node('move_arm_trajectory_client')
    '''
    move_right_arm_initial_pose()
    move_left_arm_initial_pose()
    move_head_initial_pose()
    move_right_arm_pickup_start_state()
    '''
    solve_ik()
    
    
