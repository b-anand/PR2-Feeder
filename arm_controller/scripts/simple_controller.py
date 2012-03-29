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
from arm_navigation_msgs.msg import MoveArmResult, MoveArmAction, MoveArmActionGoal, MoveArmGoal, JointConstraint, PositionConstraint, SimplePoseConstraint, OrientationConstraint, Shape
from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse, GetKinematicSolverInfo, GetKinematicSolverInfoRequest
import tf

from getch import getch

import re
import time

tf_listener = None

def get_trajectory_point(positions, time_from_start):
    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = [.0] * len(positions)
    point.time_from_start = time_from_start
    return point

def task_completed(status, result):
    #print "Succeeded" if status == GoalStatus.SUCCEEDED else "Failed"
    pass

def move_body_part(service, joints, positions):
    client = actionlib.SimpleActionClient(service, JointTrajectoryAction)
    client.wait_for_server()
    
    goal = JointTrajectoryGoal()
    goal.trajectory.joint_names = joints
    point = get_trajectory_point(positions, roslib.rostime.Duration(0))
    goal.trajectory.points.append(point)
    
    goal.trajectory.header.stamp = rospy.Time.now()
    client.send_goal(goal, done_cb=task_completed)
    #print "Goal Sent"
    client.wait_for_result()
    #print "Success."


def move_right_arm(positions):
    service = 'r_arm_controller/joint_trajectory_action'
    
    joints = [ 'r_shoulder_pan_joint', # min(right_for_robot) = -2.1, max(left) = 0.56
               'r_shoulder_lift_joint', # min(up) = -0.35, max(down) = 1.28
               'r_upper_arm_roll_joint', # min(anti_clockwise_for_robot) = -3.75, max(clockwise) = 0.65
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
    positions = [-0.001, 0.814]
    move_head(positions)

def move_right_arm_pickup_start_state():
    positions = [-0.5, -0.18, -2, -1.2, 5, -1.6, 0]
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
            return (point.x, point.y, point.z)
        else:
            print resp.detection.result
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def move_arm_completed_cb(status, result):
    if status == GoalStatus.SUCCEEDED:
        print "Succeeded"  
    else:
        print "Failed"
        print result.error_code.val


def getJointConstraints(goal):
    joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint",
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
        constraint.weight = 1.0
        joint_constraints.append(constraint)
    
    joint_constraints[0].position = -2
    joint_constraints[3].position = -0.2
    joint_constraints[5].position = -0.15
    goal.motion_plan_request.goal_constraints.joint_constraints = joint_constraints


def addPositionConstraint(goal, x, y, z):
    constraint = PositionConstraint()
    constraint.header.stamp = rospy.Time.now()
    constraint.link_name = "r_wrist_roll_link"
    constraint.header.frame_id = "base_link"
    constraint.position.x = x
    constraint.position.y = y
    constraint.position.z = z
    constraint.constraint_region_shape.type = Shape.BOX
    constraint.constraint_region_shape.dimensions.append(0.01)
    constraint.constraint_region_shape.dimensions.append(0.01)
    constraint.constraint_region_shape.dimensions.append(0.01)
    constraint.constraint_region_orientation.w = 1.0
    constraint.weight = 1.0
    goal.motion_plan_request.goal_constraints.position_constraints.append(constraint)


def addOrientationConstraint(goal):
    constraint = OrientationConstraint()
    constraint.header.stamp = rospy.Time.now()
    constraint.link_name = "r_wrist_roll_link"
    constraint.header.frame_id = "base_link"
    constraint.orientation.x = 0.0
    constraint.orientation.y = 0.0
    constraint.orientation.z = 0.0
    constraint.orientation.w = 1.0
    constraint.absolute_pitch_tolerance = 0.04
    constraint.absolute_roll_tolerance = 0.04
    constraint.absolute_yaw_tolerance = 0.04
    constraint.weight = 1.0
    goal.motion_plan_request.goal_constraints.orientation_constraints.append(constraint)

def move_arm_ompl(x, y, z):
    service = "move_right_arm"
    client = actionlib.SimpleActionClient(service, MoveArmAction)
    client.wait_for_server()
    
    goal = MoveArmGoal()
    goal.disable_ik = True
    goal.planner_service_name = "ompl_planning/plan_kinematic_path"
    goal.motion_plan_request.group_name = "right_arm"
    goal.motion_plan_request.num_planning_attempts = 5
    goal.motion_plan_request.allowed_planning_time = roslib.rostime.Duration(5)
    goal.motion_plan_request.planner_id = ""

    addPositionConstraint(goal, x, y, z)
    addOrientationConstraint(goal)
    
    '''
    constraint = JointConstraint()
    constraint.joint_name = "r_upper_arm_roll_joint"
    constraint.position = -2
    constraint.tolerance_above = 0.1
    constraint.tolerance_below = 0.1
    constraint.weight = 1.0
    goal.motion_plan_request.goal_constraints.joint_constraints.append(constraint)
    
    '''
    constraint = JointConstraint()
    constraint.joint_name = "r_shoulder_lift_joint"
    constraint.position = 0.4
    constraint.tolerance_above = 0.2
    constraint.tolerance_below = 0.2
    constraint.weight = 1.0
    goal.motion_plan_request.goal_constraints.joint_constraints.append(constraint)
    
    
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

def solve_ik(x, y, z):
    '''
    Need to call move_arm_ompl before calling this, otherwise it doesn't work. 
    '''
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
        req.ik_request.pose_stamped.header.frame_id = "base_link"

        req.ik_request.pose_stamped.pose.position.x = x
        req.ik_request.pose_stamped.pose.position.y = y
        req.ik_request.pose_stamped.pose.position.z = z

        req.ik_request.pose_stamped.pose.orientation.x = 0.0
        req.ik_request.pose_stamped.pose.orientation.y = 0.0
        req.ik_request.pose_stamped.pose.orientation.z = 0.0
        req.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        req.ik_request.ik_seed_state.joint_state.position = [-0.5, -0.18, -2, -1.2, 5, -1.6, 0]
        req.ik_request.ik_seed_state.joint_state.name = joint_names
        
        resp = GetPositionIKResponse()
        resp = ik_solver(req)
        
        print resp.solution.joint_state.name
        print resp.solution.joint_state.position
        
        if len(resp.solution.joint_state.position) == 7:
            move_right_arm(resp.solution.joint_state.position)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def get_transform(link1, link2):
    global tf_listener
    if tf_listener is None:
        tf_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (trans, rot) = tf_listener.lookupTransform(link1, link2, rospy.Time(0))
            return trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


def get_right_wrist_position():
    return get_transform('/base_link', '/r_wrist_roll_link')
    
def get_dir(x, gx):
    return 0 if (gx - x) == 0 else (gx - x) / abs(gx - x)

def control_arm_joints():
    #move_right_arm_pickup_start_state()
    
    positions = ['-0.100', '-0.354', '-2.000', '-1.900', '-1.283', '-1.100', '6.283']
    positions = [float(x) for x in positions]
    move_right_arm(positions)
    step = 0.1
    
    stop = False
    print "Enter Controls:"
    while not stop:
        c = getch()
        c = c.lower()
        if c not in ["a", "z", "s", "x", "d", "c", "f", "v", "g", "b", "h", "n", "j", "m"]:
            print "Enter correct choice, q for exit"
            continue
        elif c == "a":
            positions[0] += step
        elif c == "z":
            positions[0] -= step
        elif c == "s":
            positions[1] += step
        elif c == "x":
            positions[1] -= step
        elif c == "d":
            positions[2] += step
        elif c == "c":
            positions[2] -= step
        elif c == "f":
            positions[3] += step
        elif c == "v":
            positions[3] -= step
        elif c == "g":
            positions[4] += step
        elif c == "b":
            positions[4] -= step
        elif c == "h":
            positions[5] += step
        elif c == "n":
            positions[5] -= step
        elif c == "j":
            positions[6] += step
        elif c == "m":
            positions[6] -= step
        elif c == "q":
            stop = True
            continue
        move_right_arm(positions)

def pick_food():
    joint_names = [  "r_shoulder_pan_joint",
                     "r_shoulder_lift_joint",
                     "r_upper_arm_roll_joint",
                     "r_elbow_flex_joint",
                     "r_forearm_roll_joint",
                     "r_wrist_flex_joint",
                     "r_wrist_roll_joint"]
    
    input_string = """
['-0.200', '-0.354', '-2.250', '-1.901', '-1.383', '-1.900', '6.283'] # pickup start state.
['-0.080', '-0.18', '-2.250', '-1.901', '-1.383', '-1.900', '6.283'] #controls the downward push  
['-0.080', '-0.18', '-2.250', '-1.901', '-1.383', '-0.75', '6.283'] # pickup food.
['-0.200', '-0.354', '-1.500', '-1.901', '-1.383', '-1.600', '6.283'] # raise hand
['-0.2', '-0.053', '-1.501', '-0.119', '-1.587', '-1.601', '6.285'] # move hand away
"""
    
    positions_list = [[float(entry[1:-1]) for entry in input_line[1:].split("]")[0].split(", ")] for input_line in input_string.split("\n") if len(input_line) > 0]
    #positions_list.insert(0, [-0.5, -0.18, -2, -1.2, 5, -1.6, 0])
    for i, positions in enumerate(positions_list):
        move_right_arm(positions)
        a = input( "Done" + str(i+1) + "enter to continue:")
        
    
if __name__ == '__main__':
    rospy.init_node('move_arm_trajectory_client')
    pick_food()
    #control_arm_joints()

    #move_right_arm_initial_pose()
    '''
    move_right_arm_initial_pose()
    move_left_arm_initial_pose()
    move_head_initial_pose()
    time.sleep(10)
    detect_objects()
    move_right_arm_pickup_start_state()
    time.sleep(10)
    move_arm_ompl()
    get_ik_info()
    time.sleep(10)
    '''
    #move_right_arm_initial_pose()
    
    #get_ik_info()
    #move_right_arm_pickup_start_state()

    """
    #time.sleep(10)
    
    '''
    dx = get_dir(x, gx)
    dy = get_dir(y, gy)
    dz = get_dir(z, gz)
    print dx, dy, dz
    
    step = 0.02
    nx, ny, nz = x, y + step, z
    print nx, ny, nz
    '''
    x, y, z = get_right_wrist_position()
    print "wrist", x, y, z
    
    gx, gy, gz = detect_objects()
    print "objects", gx, gy, gz
    
    nx, ny, nz = gx, gy, gz
    move_arm_ompl(x, y - 0.02m, z)
        
    
    #time.sleep(10)
    #solve_ik(0.655698537827, 0.0111701283604, 0.523343801498)
    #move_right_arm_pickup_start_state()
    #time.sleep(10)
    #x, y, z = detect_objects()
    #print x, y, z
    """
