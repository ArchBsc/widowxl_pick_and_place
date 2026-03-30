#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance=0.01):
    if isinstance(goal, list):
        return all(abs(a - g) < tolerance for a, g in zip(actual, goal))
    return False

def compute_inverse_kinematics():
    rospy.init_node("inverse_kinematics_solver", anonymous=True)
    moveit_commander.roscpp_initialize([])
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm_manipulator"  # Adjust based on your SRDF MoveIt group
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_path_constraints(None)  # Remove constraints
    move_group.set_planning_time(10.0)  # Increase planning time
    move_group.set_num_planning_attempts(10)  # Allow multiple attempts
    move_group.set_max_velocity_scaling_factor(1.0)
    move_group.set_max_acceleration_scaling_factor(1.0)
    move_group.set_goal_tolerance(0.01)
    move_group.allow_replanning(True)
    
    print "Robot group set to:", move_group.get_name()
    
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.1  # sleep position
    target_pose.position.y = 0.0
    target_pose.position.z = 0.34
    target_pose.orientation.x = 0.0
    target_pose.orientation.y = -0.366
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 0.93
    
    move_group.set_pose_target(target_pose)
    move_group.set_start_state_to_current_state()  # Ensure valid start state
    
    plan = move_group.plan()
    if plan and plan.joint_trajectory.points:  # Check if the trajectory contains points
        move_group.execute(plan, wait=True)  # Explicit execution
        print " Plan executed successfully!"
    else:
        print " MoveIt failed to generate a valid plan"
    
    move_group.stop()
    move_group.clear_pose_targets()
    
    joint_values = move_group.get_current_joint_values()
    print "Computed Joint Values:", joint_values
    print "Planning Frame:", move_group.get_planning_frame()
    print "End Effector Link:", move_group.get_end_effector_link()
    print "Current End-Effector Pose:", move_group.get_current_pose().pose
    print "Available Planners:", move_group.get_interface_description()
    print "Robot Model Loaded:", move_group.get_name()
    print "Joint Names:", move_group.get_active_joints()
    
    moveit_commander.roscpp_shutdown()
    return joint_values

if __name__ == "__main__":
    try:
        joint_angles = compute_inverse_kinematics()
        print "Resulting Joint Angles:", joint_angles
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

