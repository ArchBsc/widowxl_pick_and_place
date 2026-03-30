#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('widowxl_pick_and_place', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm_manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    rospy.sleep(2)  # Χρόνος για αρχικοποίηση

    rospy.loginfo("Robot Groups: %s", robot.get_group_names())
    rospy.loginfo("Planning frame: %s", move_group.get_planning_frame())
    rospy.loginfo("End effector link: %s", move_group.get_end_effector_link())

    # Μετακίνηση σε home position
    try:
        move_group.set_named_target("home")
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        rospy.loginfo("Moved to home position.")
    except:
        rospy.logwarn("Home position not defined or failed to move.")

    # *** ΒΕΛΤΙΩΣΕΙΣ ΓΙΑ ΤΟ KDL SOLVER ***
    move_group.set_planning_time(30)
    move_group.set_num_planning_attempts(100)
    move_group.allow_replanning(True)
    move_group.set_goal_tolerance(0.1)
    move_group.set_goal_joint_tolerance(0.1)
    move_group.set_goal_orientation_tolerance(0.1)
    move_group.set_workspace([-1, -1, -1, 1, 1, 1])  
    move_group.set_planner_id("RRTConnectkConfigDefault")  
    move_group.set_start_state_to_current_state()  

    # *** PICK POSE ***
    pick_pose = geometry_msgs.msg.Pose()
    pick_pose.position.x = 0.29
    pick_pose.position.y = -0.002
    pick_pose.position.z = 0.34
    pick_pose.orientation.x = -0.00532360539478
    pick_pose.orientation.y = 0.0145884932619
    pick_pose.orientation.z = -0.00298934990445
    pick_pose.orientation.w = 0.999874941619
    rospy.loginfo("Current EE Position: %s", move_group.get_current_pose().pose)
    rospy.loginfo("Target Pick Position: %s", pick_pose)

    # *** CHECK IK SOLVER ΧΡΗΣΙΜΟΠΟΙΩΝΤΑΣ ΤΗΝ ΥΠΗΡΕΣΙΑ ΤΟΥ MOVEIT ***
    rospy.wait_for_service('/compute_ik')
    compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)

    ik_request = GetPositionIKRequest()
    ik_request.ik_request.group_name = "arm_manipulator"
    ik_request.ik_request.pose_stamped.header.frame_id = move_group.get_planning_frame()
    ik_request.ik_request.pose_stamped.pose = pick_pose

    try:
        ik_response = compute_ik(ik_request)
        if ik_response.error_code.val == 1:  # SUCCESS
            rospy.loginfo("IK Solution Found!")
        else:
            rospy.logerr("IK Failed! Error Code: %d", ik_response.error_code.val)
            return
    except rospy.ServiceException as e:
        rospy.logerr("IK Service call failed: %s", str(e))
        return
    move_group.set_planner_id("RRTConnectkConfigDefault")

    # Σταδιακή κίνηση (waypoints)
    waypoints = []
    start_pose = move_group.get_current_pose().pose
    waypoints.append(start_pose)

    mid_pose = geometry_msgs.msg.Pose()
    mid_pose.position.x = (start_pose.position.x + pick_pose.position.x) / 2
    mid_pose.position.y = (start_pose.position.y + pick_pose.position.y) / 2
    mid_pose.position.z = (start_pose.position.z + pick_pose.position.z) / 2
    mid_pose.orientation = start_pose.orientation  
    waypoints.append(mid_pose)

    waypoints.append(pick_pose)

    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

    if fraction > 0.9:
        rospy.loginfo("Cartesian path successful! Executing pick motion...")
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
    else:
        rospy.logerr("Pick motion failed! Only %f reached", fraction)
        return

    # *** PLACE POSE ***
    place_pose = geometry_msgs.msg.Pose()
    place_pose.position.x = 0.30
    place_pose.position.y = 0.10
    place_pose.position.z = 0.32
    place_pose.orientation.w = 1.0

    rospy.loginfo("Target Place Position: %s", place_pose)

    waypoints = []
    waypoints.append(pick_pose)

    mid_pose.position.x = (pick_pose.position.x + place_pose.position.x) / 2
    mid_pose.position.y = (pick_pose.position.y + place_pose.position.y) / 2
    mid_pose.position.z = (pick_pose.position.z + place_pose.position.z) / 2
    waypoints.append(mid_pose)

    waypoints.append(place_pose)

    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

    if fraction > 0.9:
        rospy.loginfo("Cartesian path successful! Executing place motion...")
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
    else:
        rospy.logerr("Place motion failed! Only %f reached", fraction)

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()

