#!/usr/bin/env python
import roslib
roslib.load_manifest('joint_states_listener')
import rospy
from sensor_msgs.msg import JointState

import time

right_joint_names =  ["r_shoulder_pan_joint",
                "r_shoulder_lift_joint",
                "r_upper_arm_roll_joint",
                "r_elbow_flex_joint",
                "r_forearm_roll_joint",
                "r_wrist_flex_joint",
                "r_wrist_roll_joint"]

left_joint_names =  ["l_shoulder_pan_joint",
                "l_shoulder_lift_joint",
                "l_upper_arm_roll_joint",
                "l_elbow_flex_joint",
                "l_forearm_roll_joint",
                "l_wrist_flex_joint",
                "l_wrist_roll_joint"]


def joint_state_callback(msg):
    print ["%0.3f" % msg.position[i] for joint in left_joint_names for i, name in enumerate(msg.name) if name == joint]
    

if __name__ == '__main__':
    rospy.init_node('joint_states_listener')
    print right_joint_names
    rospy.Subscriber('joint_states', JointState, joint_state_callback)
    rospy.spin()
