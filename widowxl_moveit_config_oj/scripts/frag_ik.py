#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import math

class WidowXLArmController:
    def __init__(self):
        rospy.init_node('widowxl_ik_controller', anonymous=True)
        
        self.move_group = MoveGroupCommander("arm_manipulator")
        self.arm_pose_pub = rospy.Publisher('/widowxl_arm_pose', Pose, queue_size=10)
        self.color_pub = rospy.Publisher('color_cube_topic', String, queue_size=10)
        
        self.publish_rate = rospy.Rate(5)
        rospy.Timer(rospy.Duration(0.1), self.continuous_publish_arm_pose)  
        
        self.move_group.set_named_target("home")
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.Subscriber("color_cube_topic", String, self.color_callback)
        
        rospy.Timer(rospy.Duration(5), self.test_publish_colors)
    
    def color_callback(self, color_msg):
        color = color_msg.data.lower()
        rospy.loginfo("Received color: %s" % color)
        
        available_targets = self.move_group.get_named_targets()
        target_name = color + "_position"
        
        if target_name in available_targets:
            rospy.loginfo("Moving to target: %s" % target_name)
            self.move_to_pose(target_name, 0.5, 0.5)
        else:
            rospy.logwarn("Named target '%s' does not exist! Using fallback Cartesian pose." % target_name)
            fallback_pose = Pose()
            fallback_pose.position.x = 0.15
            fallback_pose.position.y = -0.15 if color == "blue" else 0.15
            fallback_pose.position.z = 0.2
            quaternion = quaternion_from_euler(0, math.radians(90), 0)
            fallback_pose.orientation.x = quaternion[0]
            fallback_pose.orientation.y = quaternion[1]
            fallback_pose.orientation.z = quaternion[2]
            fallback_pose.orientation.w = quaternion[3]
            
            self.move_to_pose(fallback_pose, 0.2, 0.2)
    
    def test_publish_colors(self, event):
        for color in ["Red", "Green", "Blue"]:
            rospy.loginfo("Publishing test color: %s" % color)
            self.color_pub.publish(color)
            rospy.sleep(5)
    
    def continuous_publish_arm_pose(self, event):
        current_pose = self.move_group.get_current_pose().pose
        self.arm_pose_pub.publish(current_pose)
        rospy.loginfo("Published arm pose: %s" % current_pose)
    
    def move_to_pose(self, pose, velocity_scaling, acceleration_scaling):
        rospy.loginfo("Attempting to move to pose: %s" % pose)
        
        if isinstance(pose, str):  # Named target
            rospy.loginfo("Using named target: %s" % pose)
            self.move_group.set_named_target(pose)
        elif isinstance(pose, Pose):  # Pose object
            rospy.loginfo("Using Pose object")
            self.move_group.set_pose_target(pose)
        else:
            rospy.logerr("Invalid pose format! Expected a named target (string) or a Pose() object, but received: %s" % type(pose))
            return
        
        rospy.loginfo("Active Joints: %s" % self.move_group.get_active_joints())
        rospy.loginfo("Current Joint Values: %s" % self.move_group.get_current_joint_values())
        
        joint_limits = {
            "waist": (-3.5, 3.5),
            "shoulder": (-3.14, 3.14),
            "elbow": (-3.14, 3.14),
            "wrist_angle": (-3.14, 3.14),
            "wrist_rotate": (-3.14, 3.14),
        }
        rospy.loginfo("Joint Limits: %s" % joint_limits)
        
        self.move_group.set_start_state_to_current_state()
        self.move_group.set_workspace([-2, -2, -2, 2, 2, 2])
        self.move_group.set_planning_time(20)
        self.move_group.set_num_planning_attempts(15)
        self.move_group.set_goal_tolerance(0.002)
        self.move_group.set_max_velocity_scaling_factor(0.05)
        self.move_group.set_max_acceleration_scaling_factor(0.05)
        self.move_group.set_planner_id("RRTConnectkConfigDefault")
        rospy.loginfo("Attempting to move to pose: %s" % pose)
        
        if isinstance(pose, str):  # Named target
            rospy.loginfo("Using named target: %s" % pose)
            self.move_group.set_named_target(pose)
        elif isinstance(pose, Pose):  # Pose object
            rospy.loginfo("Using Pose object")
            self.move_group.set_pose_target(pose)
        else:
            rospy.logerr("Invalid pose format! Expected a named target (string) or a Pose() object, but received: %s" % type(pose))
            return
        
        rospy.loginfo("Active Joints: %s" % self.move_group.get_active_joints())
        rospy.loginfo("Current Joint Values: %s" % self.move_group.get_current_joint_values())
        
        joint_limits = {
            "waist": (-3.5, 3.5),
            "shoulder": (-3.14, 3.14),
            "elbow": (-3.14, 3.14),
            "wrist_angle": (-3.14, 3.14),
            "wrist_rotate": (-3.14, 3.14),
        }
        rospy.loginfo("Joint Limits: %s" % joint_limits)
        
        self.move_group.set_start_state_to_current_state()
        self.move_group.set_workspace([-2, -2, -2, 2, 2, 2])
        self.move_group.set_planning_time(15)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_goal_tolerance(0.005)
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        self.move_group.set_planner_id("RRTConnectkConfigDefault")
        
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if not success:
            rospy.logerr("MoveIt! failed to execute motion. Check collision settings and joint limits.")
    

def main():
    arm_controller = WidowXLArmController()
    rospy.spin()

if __name__ == '__main__':
    main()

