#!/usr/bin/env python
'''
Created on Mar 26, 2012

Adapted from: http://www.ros.org/wiki/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action

@author: ab2283
'''

import roslib; roslib.load_manifest('arm_controller')
import rospy
import actionlib
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, SingleJointPositionAction, SingleJointPositionGoal, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from actionlib_msgs.msg._GoalStatus import GoalStatus
from ee_cart_imped_action import EECartImpedClient
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest, TabletopDetectionResponse
from arm_navigation_msgs.msg import MoveArmResult, MoveArmAction, MoveArmActionGoal, MoveArmGoal, JointConstraint, PositionConstraint, SimplePoseConstraint, OrientationConstraint, Shape
from geometry_msgs.msg import Twist
from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse, GetKinematicSolverInfo, GetKinematicSolverInfoRequest
import tf

from getch import getch

import re
import time
from math import sqrt
import numpy as np

tf_listener = None
arm_move_duration = 0
robot_base_pub = None
torso_pub = None 

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
    global arm_move_duration
    client = actionlib.SimpleActionClient(service, JointTrajectoryAction)
    client.wait_for_server()
    
    goal = JointTrajectoryGoal()
    goal.trajectory.joint_names = joints
    point = get_trajectory_point(positions, roslib.rostime.Duration(arm_move_duration))
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
    '''
    TODO: Need to figure out the target frame in which the cluster point co-ordinates are returned.
    '''
    service_name = '/object_detection'
    rospy.wait_for_service(service_name)
    try:
        table_object_detector = rospy.ServiceProxy(service_name, TabletopDetection)
        
        req = TabletopDetectionRequest()
        req.return_clusters = False
        req.return_models = True
        req.num_models = 1
        
        resp = TabletopDetectionResponse()
        resp = table_object_detector(req)
        if resp.detection.result == resp.detection.SUCCESS:
            points = resp.detection.clusters[0].points
            x_points = [point.x for point in points]
            y_points = [point.y for point in points]
            z_points = [point.z for point in points]
            x = (min(x_points) + max(x_points)) / 2.0
            y = (min(y_points) + max(y_points)) / 2.0
            z = (min(z_points) + max(z_points)) / 2.0
            return x, y, z
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
    return get_transform('/base_footprint', '/r_gripper_l_finger_tip_link')
    
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

def get_spoon_location(a, b, d):
    a = np.array(a)
    b = np.array(b)
    t = a - b
    return b - d * t / np.linalg.norm(t)

def get_spoon_lowest_position(positions_list = None, spoon_length = 0.1):
    if positions_list is None:
        positions_list = [
        [-0.5, -0.18, -2, -1.2, 5, -1.6, 0], # initial state
        [-0.200, -0.354, -2.250, -1.901, -1.383, -1.6, 6.283], # pickup start state.
        [-0.080, -0.18, -2.250, -1.901, -1.383, -1.6, 6.283], #controls the downward push  
        [-0.080, -0.18, -2.250, -1.901, -1.383, -0.75, 6.283], # pickup food.
        [-0.200, -0.354, -1.500, -1.901, -1.383, -1.600, 6.283], # raise hand
        [-0.2, -0.053, -1.501, -0.119, -1.587, -1.601, 6.285], # move hand away
        ]
    move_right_arm(positions_list[1])
    a = get_right_wrist_position()
    print a
    move_right_arm(positions_list[2])
    b = get_right_wrist_position()
    print b
    return get_spoon_location(a, b, spoon_length)

def move_body_torso(position):
    service = 'torso_controller/position_joint_action'
    goal = SingleJointPositionGoal()
    goal.position = position
    goal.min_duration = roslib.rostime.Duration(2.0)
    goal.max_velocity = 1.0
     
    client = actionlib.SimpleActionClient(service, SingleJointPositionAction)
    client.send_goal(goal, done_cb=task_completed)
    client.wait_for_result()

def move_robot_base(x, y, z):
    '''
    each step will move the robot by 0.04 in that direction.
    '''
    global robot_base_pub
    msg = Twist()
    msg.linear.x = x
    msg.linear.y = y
    msg.linear.z = z
    robot_base_pub.publish(msg)

def move_torso_position(position):
    global torso_pub
    traj = JointTrajectory()
    traj.header.stamp = rospy.get_rostime()
    traj.joint_names.append("torso_lift_joint");
    traj.points.append(JointTrajectoryPoint())
    traj.points[0].positions.append(position)
    traj.points[0].velocities.append(1)
    traj.points[0].time_from_start = rospy.Duration(0.25)
    torso_pub.publish(traj)
    
    sz = 0.8 # start state hieght of torso_lift_link
    cz = get_torso_position()[2]
    epsilon = 0.02
    while abs(abs(cz - sz) - position) > epsilon:
        cz = get_torso_position()[2]
        rospy.sleep(0.2)
    
    
def get_torso_position():
    return get_transform('/odom_combined', '/torso_lift_link')

def get_robot_position():
    return get_transform('/odom_combined', '/base_footprint')

def get_move_step(x, gx):
    return 0 if abs(gx - x) <= 0.03 else (gx - x) / abs(gx - x)

def pick_food():
    positions_list = [
        [-0.5, -0.18, -2, -1.2, 5, -1.6, 0], # initial state
        [-0.200, -0.354, -2.250, -1.901, -1.383, -1.6, 6.283], # pickup start state.
        [-0.080, -0.18, -2.250, -1.901, -1.383, -1.6, 6.283], #controls the downward push  
        [-0.080, -0.18, -2.250, -1.901, -1.383, -0.75, 6.283], # pickup food.
        [-0.200, -0.354, -1.500, -1.901, -1.383, -1.600, 6.283], # raise hand
        [-0.2, -0.053, -1.501, -0.119, -1.587, -1.601, 6.285], # move hand away
        ]
    
    for positions in positions_list:
        move_right_arm(positions)
        rospy.sleep(2)
    
def initialize_robot_position():
    spoon_pos = [ 0.39501852, -0.0907355,   0.45095445] # obtained by calling get_spoon_lowest_position with spoon_length = 0.1
    
    tx, ty, tz = detect_objects()
    print tx, ty, tz
    current_pos = get_robot_position()
    cx, cy, cz = current_pos
    print "start", current_pos
    target_pos = (cx + tx - spoon_pos[0], cy + ty - spoon_pos[1], cz)
    print "target", target_pos
    tp = np.array(target_pos)
    cp = np.array(current_pos)
    
    while True:
        xstep = get_move_step(cp[0], tp[0])
        ystep = get_move_step(cp[1], tp[1])
        if xstep == 0 and ystep == 0:
            print "Reached near the goal."
            break
        move_robot_base(xstep, ystep, 0)
        rospy.sleep(1)
        cp = np.array(get_robot_position())
        print cp
    
    move_torso_position(tz - spoon_pos[2] + 0.05) # TODO: offset of 5 cms, needs to be changed based on actual spoon size.
    print "Moved torso to final pos."    
        
def initialize_body():
    move_right_arm_initial_pose()
    move_left_arm_initial_pose()
    move_head_initial_pose()
    print "Initialized robot body."
    rospy.sleep(8)
    
if __name__ == '__main__':
    '''
    TODO: 
    1. Change arm_move_duration to 5 before running on pr2.
    2. Velocity of torso_controller needs to be set properly before running on pr2.
    '''
    robot_base_pub = rospy.Publisher("base_controller/command", Twist)
    torso_pub= rospy.Publisher('torso_controller/command', JointTrajectory)
    rospy.init_node('move_arm_trajectory_client')
    initialize_body()
    initialize_robot_position()
    pick_food()
