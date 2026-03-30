#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
from math import pi

def simple_pick_and_place():
    # 1. Αρχικοποίηση
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place', anonymous=True)

    # 2. Ορισμός των Planning Groups (ΒΑΣΕΙ ΤΟΥ SRDF ΣΟΥ)
    # ΠΡΟΣΟΧΗ: Αν το robot_description είναι σε namespace (π.χ. /wx250s/robot_description),
    # ίσως χρειαστεί να τρέξεις το script μέσα στο namespace ή να ορίσεις ns="/..."
    try:
        arm_group = moveit_commander.MoveGroupCommander("widowx_arm")
        gripper_group = moveit_commander.MoveGroupCommander("widowx_gripper")
    except RuntimeError:
        print("CRITICAL ERROR: Δεν βρέθηκε το robot_description.")
        print("Βεβαιώσου ότι έχεις τρέξει το demo.launch ή το moveit launch file.")
        return

    # Ρυθμίσεις ταχύτητας για ασφάλεια (0.5 = 50% ταχύτητα)
    arm_group.set_max_velocity_scaling_factor(0.5)
    arm_group.set_max_acceleration_scaling_factor(0.5)

    # Καθαρισμός προηγούμενων εντολών
    arm_group.clear_pose_targets()

    # --- ΣΥΝΤΕΤΑΓΜΕΝΕΣ ΣΤΟΧΟΥ (F20_20_B) ---
    # Βάλε το αντικείμενο 30cm μπροστά από τη βάση
    object_location = {'x': 0.30, 'y': 0.00, 'z': 0.02} 
    drop_location = {'x': 0.20, 'y': 0.15, 'z': 0.05}

    print("--- Βήμα 1: Πάμε στο 'default_pose' ---")
    arm_group.set_named_target("default_pose")
    arm_group.go(wait=True)
    
    print("--- Βήμα 2: Άνοιγμα Gripper ---")
    gripper_group.set_named_target("gripper_open_pose")
    gripper_group.go(wait=True)

    # --- Βήμα 3: Προσέγγιση (PRE-GRASP) ---
    print("--- Βήμα 3: Προσέγγιση ---")
    pose_goal = geometry_msgs.msg.Pose()
    
    # ΠΡΟΣΑΝΑΤΟΛΙΣΜΟΣ:
    # Πρέπει να πειραματιστείς εδώ. Συνήθως για να κοιτάει κάτω:
    # Αν ο καρπός σου είναι περίεργος, δοκίμασε q=[0, 1, 0, 0] ή q=[0.707, 0.707, 0, 0]
    # Ξεκινάμε με τυπικό orientation προς τα κάτω:
    pose_goal.orientation.w = 0.0
    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 1.0
    pose_goal.orientation.z = 0.0
    
    pose_goal.position.x = object_location['x']
    pose_goal.position.y = object_location['y']
    pose_goal.position.z = object_location['z'] + 0.15  # 15cm πάνω

    arm_group.set_pose_target(pose_goal)
    # Θέτουμε το reference frame
    arm_group.set_pose_reference_frame("arm_base_link") # Βάση SRDF
    
    plan = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    # --- Βήμα 4: Κατέβασμα (GRASP) ---
    print("--- Βήμα 4: Κατέβασμα ---")
    pose_goal.position.z = object_location['z']
    
    waypoints = []
    waypoints.append(copy.deepcopy(pose_goal))
    # compute_cartesian_path(waypoints, eef_step, jump_threshold)
    (plan, fraction) = arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    
    # Αν το cartesian path αποτύχει (fraction < 1.0), μην το εκτελέσεις
    if fraction < 0.9:
        print(f"Προσοχή: Υπολογίστηκε μόνο το {fraction*100}% της διαδρομής. Ακύρωση.")
    else:
        arm_group.execute(plan, wait=True)

        # --- Βήμα 5: Κλείσιμο Gripper ---
        print("--- Βήμα 5: Πιάσιμο ---")
        gripper_group.set_named_target("gripper_closed_pose")
        gripper_group.go(wait=True)
        rospy.sleep(1.0)

        # --- Βήμα 6: Ανύψωση (LIFT) ---
        print("--- Βήμα 6: Ανύψωση ---")
        pose_goal.position.z = object_location['z'] + 0.15
        arm_group.set_pose_target(pose_goal)
        arm_group.go(wait=True)

        # --- Βήμα 7: Μεταφορά ---
        print("--- Βήμα 7: Μεταφορά ---")
        pose_goal.position.x = drop_location['x']
        pose_goal.position.y = drop_location['y']
        pose_goal.position.z = drop_location['z'] + 0.15
        arm_group.set_pose_target(pose_goal)
        arm_group.go(wait=True)

        # --- Βήμα 8: Απόθεση ---
        print("--- Βήμα 8: Απόθεση ---")
        gripper_group.set_named_target("gripper_open_pose")
        gripper_group.go(wait=True)

    print("--- Τέλος Προγράμματος ---")
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        simple_pick_and_place()
    except rospy.ROSInterruptException:
        pass
