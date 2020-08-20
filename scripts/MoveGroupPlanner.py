#!/usr/bin/env python
from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander

import actionlib

import std_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg
import tf_conversions
import math
import franka_control.srv
from math import pi
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_matrix, rotation_matrix, translation_from_matrix, quaternion_matrix
import tf
from tf.listener import TransformerROS, Transformer
import numpy as np

from whole_part import SceneObject

import yaml
import tf2_ros
class MoveGroupPlanner():
    def __init__(self):
        ### MoveIt! 
        moveit_commander.roscpp_initialize(sys.argv)
        br = tf.TransformBroadcaster()
    

        #rospy.init_node('move_group_planner',
        #                anonymous=True)
 
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.plan_scene = moveit_commander.PlanningScene()
        self.group_1st = moveit_commander.MoveGroupCommander("panda_left")
        self.group_2nd = moveit_commander.MoveGroupCommander("panda_right")
        self.group_3rd = moveit_commander.MoveGroupCommander("panda_top")
        self.group_list = [self.group_1st, self.group_2nd, self.group_3rd]
        self.group_chain = moveit_commander.MoveGroupCommander("panda_closed_chain")
        self.group_chairup = moveit_commander.MoveGroupCommander("panda_chair_up")
        self.hand_left = moveit_commander.MoveGroupCommander("hand_left")
        self.hand_right = moveit_commander.MoveGroupCommander("hand_right")
        self.hand_top = moveit_commander.MoveGroupCommander("hand_top")
        self.hand_list = [self.hand_left, self.hand_right, self.hand_top]
        
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.group_1st.get_planning_frame()
        print ("============ Reference frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.group_1st.get_end_effector_link()
        print ("============ End effector: %s" % self.eef_link)

        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.group_3rd.get_planning_frame()
        print ("============ Reference frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.group_3rd.get_end_effector_link()
        print ("============ End effector: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print ("============ Robot Groups:", self.robot.get_group_names())

        rospy.sleep(1)
        self.stefan = SceneObject()

        # self.scene.remove_attached_object(self.group_1st.get_end_effector_link())
        # self.scene.remove_attached_object(self.group_3rd.get_end_effector_link())
        # self.scene.remove_world_object()

        ### Franka Collision
        self.set_collision_behavior = rospy.ServiceProxy(
            'franka_control/set_force_torque_collision_behavior',
            franka_control.srv.SetForceTorqueCollisionBehavior)
        #self.set_collision_behavior.wait_for_service()

        self.active_controllers = []

        self.listener = tf.TransformListener()
        self.tr = TransformerROS()

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.8
        box_pose.pose.position.z = 0.675        
        
        box_pose2 = geometry_msgs.msg.PoseStamped()
        box_pose2.header.frame_id = "base"
        box_pose2.pose.orientation.w = 1.0
        box_pose2.pose.position.x = 0.8
        box_pose2.pose.position.y = -0.25
        box_pose2.pose.position.z = 1.0
       
        self.scene.add_box("box", box_pose, size = (1.2, 0.5, 0.15))
        # self.scene.add_box("box2", box_pose2, size = (0.3, 0.08, 0.1))
        


    # geometry_msgs.msg.Pose() or self.group.get_current_joint_values()
    def plan(self, goal, arm_name):
        if (arm_name == '1st'):
            self.group_1st.set_max_velocity_scaling_factor = 0.6
            self.group_1st.set_max_acceleration_scaling_factor = 0.4
            self.group_1st.set_start_state_to_current_state()
            trajectory = self.group_1st.plan(goal)
            self.group_1st.go()
        if (arm_name == '3rd'):
            self.group_3rd.set_max_velocity_scaling_factor = 0.6
            self.group_3rd.set_max_acceleration_scaling_factor = 0.4
            self.group_3rd.set_start_state_to_current_state()
            trajectory = self.group_3rd.plan(goal) 
        if (arm_name == '2nd'):
            self.group_2nd.set_max_velocity_scaling_factor = 0.6
            self.group_2nd.set_max_acceleration_scaling_factor = 0.4
            self.group_2nd.set_start_state_to_current_state()
            trajectory = self.group_2nd.plan(goal)        
        
    
    def plan_cartesian_target(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(math.radians(90), math.radians(90), math.radians(0)))
        pose_goal.position.x =  0.5429
        pose_goal.position.y = 0.05
        pose_goal.position.z = 0.6 + 0.30

        trajectory = self.group_1st.plan(pose_goal)
        return trajectory

    def initial_pose(self, arm="all"):
        joint_goal = [0, -0.785, 0, -1.571, 0, 1.571, 0.785]
        if (arm == "1st"):
            self.group_1st.plan(joint_goal)
            self.group_1st.go()
        elif (arm == "2nd"):
            self.group_2nd.plan(joint_goal)
            self.group_2nd.go()
        elif (arm == "3rd"):
            self.group_3rd.plan(joint_goal)
            self.group_3rd.go()
        else :
            for group in self.group_list:
                group.plan(joint_goal)
                group.go()


    def plan_joint_target(self, joint_goal, arm_name='panda_closed_chain'):
        if (len(joint_goal) == 14):
            if (arm_name=='panda_chair_up'):
                self.group_chairup.plan(joint_goal)
                self.group_chairup.go()
            else:
                self.group_chain.plan(joint_goal)
                self.group_chain.go()

        elif (arm_name=='1st'):
            self.group_1st.plan(joint_goal)
            self.group_1st.go()
        elif (arm_name=='3rd'):
            self.group_3rd.plan(joint_goal)
            self.group_3rd.go()
        
       
    def gripper_open(self, arm="all"):
        joint_goal = self.hand_left.get_current_joint_values()
        joint_goal[0] = 0.038
        joint_goal[1] = 0.038
        if (arm == "1st"):
            self.hand_left.plan(joint_goal)
            print("OPEN 1ST GRIPPER")
            self.hand_left.go()
        elif (arm == "3rd"):
            self.hand_top.plan(joint_goal)
            print("OPEN 3RD GRIPPER")
            self.hand_top.go()
        elif (arm == "2nd"):
            self.hand_right.plan(joint_goal)
            print("OPEN 2ND GRIPPER")
            self.hand_right.go()
        else :
            for hand in self.hand_list:
                hand.plan(joint_goal)
                hand.go()


    def gripper_close(self):
        joint_goal = self.hand_left.get_current_joint_values()
        joint_goal[0] = 0.0
        joint_goal[1] = 0.0
        trajectory = self.hand_left.plan(joint_goal)
        return trajectory

    def display_trajectory(self, plan):
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)