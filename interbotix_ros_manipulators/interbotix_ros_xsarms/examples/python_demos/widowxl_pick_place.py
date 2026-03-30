#!/usr/bin/env python3

"""
WidowXL Pick-and-Place με Interbotix Python API
 
Workflow:
1. Robot πηγαίνει σε home pose (Γ shape, camera κοιτάει κάτω)
2. Mock camera δίνει coordinates του object
3. Robot υπολογίζει trajectory και πιάνει το object
4. Robot το αφήνει σε drop location

Usage:
    roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=widowxl
    python3 widowxl_pick_place.py
"""

import rospy
# from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from geometry_msgs.msg import PoseStamped, Point
import numpy as np
from math import pi, atan2, sqrt
import sys


class WidowXLPickPlace:
    def __init__(self):
        """Initialize WidowXL pick-and-place controller"""
        
        # Initialize Interbotix Robot
        self.bot = InterbotixManipulatorXS(
            robot_model="widowxl",
            group_name="arm",
            gripper_name="gripper"
        )
        
        rospy.loginfo("WidowXL Pick-and-Place initialized")
        
        # Grasp configuration
        self.grasp_config = {
            'approach_height': 0.12,    # 12cm πάνω από object
            'grasp_z_offset': 0.0,      # Gripper offset
            'retreat_height': 0.15,     # 15cm ανύψωση μετά grasp
            'gripper_pressure': 0.5,    # 0.0 - 1.0
        }
        
        # Drop location (σε σχέση με base_link)
        self.drop_location = {
            'x': 0.20,
            'y': -0.15,
            'z': 0.08
        }
        
        # Workspace limits (ασφάλεια)
        self.workspace_limits = {
            'x_min': 0.10, 'x_max': 0.40,
            'y_min': -0.25, 'y_max': 0.25,
            'z_min': 0.01, 'z_max': 0.30
        }
        
        self.objects_picked = 0
        
    def go_to_home_pose(self):
        """
        Πήγαινε σε home pose (Γ shape)
        Η camera κοιτάει προς τα κάτω για scanning
        """
        rospy.loginfo("Moving to HOME pose...")
        
        # Home pose: Γ shape με camera να κοιτάει κάτω
        # Αυτές είναι παράδειγμα τιμές - ΑΛΛΑΞΕ με το δικό σου home pose
        self.bot.arm.go_to_home_pose()
        rospy.sleep(2)
        
        # Εναλλακτικά, με custom joint positions:
        # self.bot.arm.set_joint_positions([0, -0.5, 0.8, 0, 0.3])
        
        rospy.loginfo("✓ Reached HOME pose")
        
    def go_to_sleep_pose(self):
        """Πήγαινε σε sleep pose"""
        rospy.loginfo("Moving to SLEEP pose...")
        self.bot.arm.go_to_sleep_pose()
        rospy.sleep(2)
        rospy.loginfo("✓ Reached SLEEP pose")
        
    def validate_position(self, x, y, z):
        """
        Validate ότι το position είναι εντός workspace
        """
        if not (self.workspace_limits['x_min'] <= x <= self.workspace_limits['x_max']):
            rospy.logwarn(f"X={x:.3f} out of bounds!")
            return False
        if not (self.workspace_limits['y_min'] <= y <= self.workspace_limits['y_max']):
            rospy.logwarn(f"Y={y:.3f} out of bounds!")
            return False
        if not (self.workspace_limits['z_min'] <= z <= self.workspace_limits['z_max']):
            rospy.logwarn(f"Z={z:.3f} out of bounds!")
            return False
        return True
        
    def calculate_grasp_poses(self, target_x, target_y, target_z):
        """
        Υπολογίζει approach, grasp, retreat poses
        
        Args:
            target_x, target_y, target_z: Coordinates του object σε meters
            
        Returns:
            dict με approach, grasp, retreat coordinates
        """
        poses = {}
        
        # Approach pose (πάνω από το object)
        poses['approach'] = {
            'x': target_x,
            'y': target_y,
            'z': target_z + self.grasp_config['approach_height']
        }
        
        # Grasp pose (στο ύψος του object)
        poses['grasp'] = {
            'x': target_x,
            'y': target_y,
            'z': target_z + self.grasp_config['grasp_z_offset']
        }
        
        # Retreat pose (ανύψωση μετά το grasp)
        poses['retreat'] = {
            'x': target_x,
            'y': target_y,
            'z': target_z + self.grasp_config['retreat_height']
        }
        
        return poses
        
    def move_to_position(self, x, y, z, moving_time=2.0):
        """
        Κίνηση end-effector σε (x, y, z) position
        
        Args:
            x, y, z: Target position σε meters
            moving_time: Χρόνος κίνησης σε seconds
        """
        if not self.validate_position(x, y, z):
            rospy.logerr("Position validation failed!")
            return False
            
        try:
            rospy.loginfo(f"Moving to ({x:.3f}, {y:.3f}, {z:.3f})...")
            
            self.bot.arm.set_ee_pose_components(
                x=x,
                y=y,
                z=z,
                moving_time=moving_time,
                accel_time=0.3
            )
            
            rospy.sleep(moving_time + 0.5)  # Wait for motion to complete
            return True
            
        except Exception as e:
            rospy.logerr(f"Motion failed: {e}")
            return False
            
    def open_gripper(self, delay=1.0):
        """Άνοιγμα gripper"""
        rospy.loginfo("Opening gripper...")
        self.bot.gripper.open()
        rospy.sleep(delay)
        
    def close_gripper(self, delay=1.0):
        """Κλείσιμο gripper"""
        rospy.loginfo("Closing gripper...")
        self.bot.gripper.close()
        rospy.sleep(delay)
        
    def pick_object(self, object_x, object_y, object_z):
        """
        Pick sequence για object στο (x, y, z)
        
        Returns:
            True αν το pick ήταν επιτυχές
        """
        rospy.loginfo("="*60)
        rospy.loginfo(f"PICK SEQUENCE for object at ({object_x:.3f}, {object_y:.3f}, {object_z:.3f})")
        rospy.loginfo("="*60)
        
        # Validate position
        if not self.validate_position(object_x, object_y, object_z):
            rospy.logerr("Object position out of workspace!")
            return False
        
        # Calculate poses
        poses = self.calculate_grasp_poses(object_x, object_y, object_z)
        
        try:
            # Step 1: Open gripper
            self.open_gripper(delay=0.5)
            
            # Step 2: Move to approach pose
            rospy.loginfo("→ Step 1/4: Moving to approach pose...")
            if not self.move_to_position(
                poses['approach']['x'],
                poses['approach']['y'],
                poses['approach']['z'],
                moving_time=2.0
            ):
                return False
                
            # Step 3: Move down to grasp pose
            rospy.loginfo("→ Step 2/4: Moving to grasp pose...")
            if not self.move_to_position(
                poses['grasp']['x'],
                poses['grasp']['y'],
                poses['grasp']['z'],
                moving_time=1.5
            ):
                return False
                
            # Step 4: Close gripper
            rospy.loginfo("→ Step 3/4: Closing gripper...")
            self.close_gripper(delay=1.0)
            
            # Step 5: Retreat
            rospy.loginfo("→ Step 4/4: Retreating with object...")
            if not self.move_to_position(
                poses['retreat']['x'],
                poses['retreat']['y'],
                poses['retreat']['z'],
                moving_time=2.0
            ):
                return False
                
            rospy.loginfo("✓ PICK sequence completed successfully!")
            return True
            
        except Exception as e:
            rospy.logerr(f"Pick failed: {e}")
            return False
            
    def place_object(self, place_x, place_y, place_z):
        """
        Place sequence για object στο (x, y, z)
        
        Returns:
            True αν το place ήταν επιτυχές
        """
        rospy.loginfo("="*60)
        rospy.loginfo(f"PLACE SEQUENCE at ({place_x:.3f}, {place_y:.3f}, {place_z:.3f})")
        rospy.loginfo("="*60)
        
        # Validate position
        if not self.validate_position(place_x, place_y, place_z):
            rospy.logerr("Place position out of workspace!")
            return False
        
        # Calculate poses
        poses = self.calculate_grasp_poses(place_x, place_y, place_z)
        
        try:
            # Step 1: Move to approach pose
            rospy.loginfo("→ Step 1/4: Moving to place approach...")
            if not self.move_to_position(
                poses['approach']['x'],
                poses['approach']['y'],
                poses['approach']['z'],
                moving_time=2.0
            ):
                return False
                
            # Step 2: Move down to place pose
            rospy.loginfo("→ Step 2/4: Moving to place position...")
            if not self.move_to_position(
                poses['grasp']['x'],
                poses['grasp']['y'],
                poses['grasp']['z'],
                moving_time=1.5
            ):
                return False
                
            # Step 3: Release gripper
            rospy.loginfo("→ Step 3/4: Releasing gripper...")
            self.open_gripper(delay=1.0)
            
            # Step 4: Retreat
            rospy.loginfo("→ Step 4/4: Retreating...")
            if not self.move_to_position(
                poses['retreat']['x'],
                poses['retreat']['y'],
                poses['retreat']['z'],
                moving_time=2.0
            ):
                return False
                
            rospy.loginfo("✓ PLACE sequence completed successfully!")
            return True
            
        except Exception as e:
            rospy.logerr(f"Place failed: {e}")
            return False
            
    def execute_pick_and_place(self, object_coords):
        """
        Complete pick-and-place workflow
        
        Args:
            object_coords: dict με 'x', 'y', 'z' του object
        """
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("STARTING PICK-AND-PLACE WORKFLOW")
        rospy.loginfo("="*70)
        
        try:
            # Go to home
            self.go_to_home_pose()
            rospy.sleep(1)
            
            # Pick object
            success = self.pick_object(
                object_coords['x'],
                object_coords['y'],
                object_coords['z']
            )
            
            if not success:
                rospy.logerr("Pick failed! Aborting...")
                self.go_to_home_pose()
                return False
                
            # Place object
            success = self.place_object(
                self.drop_location['x'],
                self.drop_location['y'],
                self.drop_location['z']
            )
            
            if not success:
                rospy.logerr("Place failed!")
                self.go_to_home_pose()
                return False
                
            # Return to home
            self.go_to_home_pose()
            
            self.objects_picked += 1
            rospy.loginfo("="*70)
            rospy.loginfo(f"✓✓✓ WORKFLOW COMPLETE! Objects picked: {self.objects_picked}")
            rospy.loginfo("="*70 + "\n")
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Workflow error: {e}")
            self.go_to_home_pose()
            return False
            
    def shutdown(self):
        """Cleanup"""
        rospy.loginfo("Shutting down...")
        self.go_to_sleep_pose()


def main():
    """
    Main function
    """
    rospy.loginfo("Starting WidowXL Pick-and-Place Demo")
    rospy.loginfo("="*70)
    rospy.loginfo("Make sure you've launched the robot control first:")
    rospy.loginfo("  roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=widowxl")
    rospy.loginfo("="*70 + "\n")
    
    try:
        # Initialize controller
        controller = WidowXLPickPlace()
        
        # Wait για initialization
        rospy.sleep(2)
        
        # Test με hardcoded object coordinates
        # ΑΥ ΤΑ ΘΕΛΕΙΣ ΑΠΟ MOCK CAMERA - ΘΑ ΤΟ ΚΑΝΟΥΜΕ ΠΑΡΑΚΑΤΩ
        test_object = {
            'x': 0.25,   # 25cm μπροστά από base
            'y': 0.10,   # 10cm δεξιά
            'z': 0.02    # 2cm ψηλά από έδαφος
        }
        
        rospy.loginfo(f"Testing with object at: x={test_object['x']}, y={test_object['y']}, z={test_object['z']}")
        rospy.loginfo("Starting in 3 seconds...")
        rospy.sleep(3)
        
        # Execute pick and place
        controller.execute_pick_and_place(test_object)
        
        rospy.loginfo("\nDemo complete! Press Ctrl+C to exit.")
        rospy.spin()
        
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        if 'controller' in locals():
            controller.shutdown()


if __name__ == '__main__':
    main()