#!/usr/bin/env python
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs

def simple_pick_place():
  ## First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('simple_pick_place',
                  anonymous=True)

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group refers to the joints of
  ## robot1. This interface can be used to plan and execute motions on robot1.
  robot1_group = moveit_commander.MoveGroupCommander("arm_manipulator")
  ## MoveGroup Commander Object for robot2.
  robot2_group = moveit_commander.MoveGroupCommander("gripper")

  ## Action clients to the ExecuteTrajectory action server.
  robot1_client = actionlib.SimpleActionClient('execute_trajectory',
    moveit_msgs.msg.ExecuteTrajectoryAction)
  robot1_client.wait_for_server()
  rospy.loginfo('Execute Trajectory server is available for robot1')
  robot2_client = actionlib.SimpleActionClient('execute_trajectory',
    moveit_msgs.msg.ExecuteTrajectoryAction)
  robot2_client.wait_for_server()
  rospy.loginfo('Execute Trajectory server is available for robot2')

  ## Set a named joint configuration as the goal to plan for a move group.
  ## Named joint configurations are the robot poses defined via MoveIt! Setup Assistant.
  robot1_group.set_named_target("home")

  ## Plan to the desired joint-space goal using the default planner (RRTConnect).
  robot1_plan_home = robot1_group.plan()
  ## Create a goal message object for the action server.
  robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
  ## Update the trajectory in the goal message.
  robot1_goal.trajectory = robot1_plan_home

  ## Send the goal to the action server.
  robot1_client.send_goal(robot1_goal)
  robot1_client.wait_for_result()
  rospy.sleep(0.5)
  robot1_group.set_named_target("sleep")
  robot1_plan_pregrasp = robot1_group.plan()
  robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
  robot1_goal.trajectory = robot1_plan_pregrasp
  robot1_client.send_goal(robot1_goal)
  robot1_client.wait_for_result()

## Cartesian Paths
  ## ^^^^^^^^^^^^^^^
  ## You can plan a cartesian path directly by specifying a list of waypoints
  ## for the end-effector to go through.
  waypoints = []
  # start with the current pose
  current_pose = robot1_group.get_current_pose()
  rospy.sleep(0.5)
  current_pose = robot1_group.get_current_pose()

 # x2 = 0.26178346748
 # x1 = 0.208798729121
 # xdiff = x2-x1
 # z2 = 0.267032607331
 # z1 = 0.131452205815
 # zdiff = z2-z1

  ## create linear offsets to the current pose
  new_eef_pose = geometry_msgs.msg.Pose()

  # Manual offsets because we don't have a camera to detect objects yet.
  new_eef_pose.position.x = current_pose.pose.position.x  
  new_eef_pose.position.y = current_pose.pose.position.y
  new_eef_pose.position.z = current_pose.pose.position.z - 0.05 
        #pregrap:
	#x: 0.208798729121
	#y: 1.74447919618e-05
	#z: 0.131452205815
	
	#default pose:
        #x: 0.26178346748
	#y: 2.54320486596e-06
	#z: 0.267032607331



  # Retain orientation of the current pose.
  new_eef_pose.orientation = copy.deepcopy(current_pose.pose.orientation)

  waypoints.append(new_eef_pose)
  waypoints.append(current_pose.pose)
  print(new_eef_pose.position)
  print(current_pose.pose.position)

 ## We want the cartesian path to be interpolated at a resolution of 1 cm
  ## which is why we will specify 0.01 as the eef_step in cartesian
  ## translation.  We will specify the jump threshold as 0.0, effectively
  ## disabling it.
  fraction = 0.0
  for count_cartesian_path in range(0,3):
    if fraction < 1.0:
      (plan_cartesian, fraction) = robot1_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
    else:
      break

  robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
  robot1_goal.trajectory = plan_cartesian
  robot1_client.send_goal(robot1_goal)
  robot1_client.wait_for_result()
  rospy.sleep(0.5)
  robot1_group.set_named_target("PREGRAP")
  robot1_plan_place = robot1_group.plan()
  robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
  robot1_goal.trajectory = robot1_plan_place
  robot1_client.send_goal(robot1_goal)
  robot1_client.wait_for_result()


  robot1_client.send_goal(robot1_goal)
  robot1_client.wait_for_result()
  rospy.sleep(0.5)
  robot1_group.set_named_target("GRAP")
  robot1_plan_pregrasp = robot1_group.plan()
  robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
  robot1_goal.trajectory = robot1_plan_pregrasp
  robot1_client.send_goal(robot1_goal)
  robot1_client.wait_for_result()

  robot1_client.send_goal(robot1_goal)
  robot1_client.wait_for_result()
  rospy.sleep(0.5)
  robot1_group.set_named_target("REVERSE")
  robot1_plan_pregrasp = robot1_group.plan()
  robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
  robot1_goal.trajectory = robot1_plan_pregrasp
  robot1_client.send_goal(robot1_goal)
  robot1_client.wait_for_result()


  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()


if __name__=='__main__':
  try:
    simple_pick_place()
  except rospy.ROSInterruptException:
    pass


