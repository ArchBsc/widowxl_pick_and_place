#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose
from tf import transformations

class WidowXlIK:
    def __init__(self, name="widowxl_ik_node", rate=10):
        rospy.init_node(name)
        self.rate = rospy.Rate(rate)
        
        # Initialize MoveIt!
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("arm_manipulator")
        
        # Ensure MoveIt! uses the correct IK solver
        rospy.set_param('/move_group/arm_manipulator/kinematics_solver', 'kdl_kinematics_plugin/KDLKinematicsPlugin')
        
        self.move_group.set_planning_time(10.0)  # Increased planning time
        self.move_group.set_num_planning_attempts(20)  # Allow more attempts
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        
        rospy.loginfo("WidowXl IK Node Initialized")
    
    def go_to_position(self, x, y, z, roll=0, pitch=0, yaw=0):
        """Moves the arm to the given (x, y, z) position using inverse kinematics."""
        rospy.loginfo("Moving to Target Position: X={}, Y={}, Z={}".format(x, y, z))
        
        pose_goal = Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        # Convert RPY to Quaternion
        quaternion = transformations.quaternion_from_euler(roll, pitch, yaw)
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        
        self.move_group.set_pose_target(pose_goal)
        
        # Debug - Print IK Solution and Joint States
        rospy.loginfo("Current Joint Values: {}".format(self.move_group.get_current_joint_values()))
        plan = self.move_group.plan()
        if plan:
            rospy.loginfo("IK Solution Found")
            self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            rospy.loginfo("Movement complete.")
        else:
            rospy.logwarn("No IK solution found! Maybe the target is unreachable?")
    
    def run(self):
        rospy.loginfo("Moving to predefined target position...")
        self.go_to_position(0.05, 0.0, 0.05)  # Change this to any target you want
        rospy.loginfo("Done. Node will now exit.")

if __name__ == "__main__":
    node = WidowXlIK()
    node.run()
