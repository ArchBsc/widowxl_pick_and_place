#!/usr/bin/env python3

"""
WidowXL Automated Pick-and-Place με Camera Integration

Αυτό το script:
1. Ακούει για object detections από το mock camera
2. Όταν εντοπίζει object, εκτελεί pick-and-place
3. Επιστρέφει σε home και περιμένει για το επόμενο object

Usage:
    # Terminal 1: Launch robot
    roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=widowxl
    
    # Terminal 2: Launch mock camera
    python3 widowxl_mock_camera.py
    
    # Terminal 3: Run this script
    python3 widowxl_pick_place_auto.py
"""

import rospy
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
import sys


class AutomatedPickPlace:
    def __init__(self):
        rospy.init_node('automated_pick_place', anonymous=True)
        
        # Initialize robot (χωρίς rospy.init_node γιατί το κάναμε πιο πάνω)
        self.bot = InterbotixManipulatorXS(
            robot_model="widowxl",
            group_name="arm",
            gripper_name="gripper"
        )
        
        # Configuration
        self.grasp_config = {
            'approach_height': 0.12,
            'grasp_z_offset': 0.0,
            'retreat_height': 0.15,
        }
        
        self.drop_location = {
            'x': 0.20,
            'y': -0.15,
            'z': 0.08
        }
        
        self.workspace_limits = {
            'x_min': 0.10, 'x_max': 0.40,
            'y_min': -0.25, 'y_max': 0.25,
            'z_min': 0.01, 'z_max': 0.30
        }
        
        # State
        self.is_busy = False
        self.objects_picked = 0
        self.last_detection = None
        
        # Subscribers
        self.object_sub = rospy.Subscriber(
            '/widowxl/detected_object',
            PoseStamped,
            self.object_detected_callback
        )
        
        self.status_sub = rospy.Subscriber(
            '/widowxl/detection_status',
            String,
            self.status_callback
        )
        
        rospy.loginfo("="*70)
        rospy.loginfo("Automated Pick-and-Place System READY")
        rospy.loginfo("Waiting for object detections from camera...")
        rospy.loginfo("="*70 + "\n")
        
        # Go to home pose
        self.go_to_home_pose()
        
    def status_callback(self, msg):
        """Callback για camera status messages"""
        # Απλά log το status
        pass
        
    def object_detected_callback(self, msg):
        """
        Callback όταν η camera εντοπίζει object
        
        Args:
            msg: PoseStamped με το object position
        """
        if self.is_busy:
            rospy.logwarn("🔴 Robot busy, ignoring detection")
            return
            
        rospy.loginfo("\n" + "🎯 "*20)
        rospy.loginfo(f"NEW OBJECT DETECTED at:")
        rospy.loginfo(f"  x = {msg.pose.position.x:.3f} m")
        rospy.loginfo(f"  y = {msg.pose.position.y:.3f} m")
        rospy.loginfo(f"  z = {msg.pose.position.z:.3f} m")
        rospy.loginfo("🎯 "*20 + "\n")
        
        # Store detection
        self.last_detection = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
        
        # Execute pick and place
        self.execute_pick_and_place(self.last_detection)
        
    def validate_position(self, x, y, z):
        """Validate position εντός workspace"""
        if not (self.workspace_limits['x_min'] <= x <= self.workspace_limits['x_max']):
            rospy.logwarn(f"❌ X={x:.3f} out of bounds!")
            return False
        if not (self.workspace_limits['y_min'] <= y <= self.workspace_limits['y_max']):
            rospy.logwarn(f"❌ Y={y:.3f} out of bounds!")
            return False
        if not (self.workspace_limits['z_min'] <= z <= self.workspace_limits['z_max']):
            rospy.logwarn(f"❌ Z={z:.3f} out of bounds!")
            return False
        return True
        
    def go_to_home_pose(self):
        """Πήγαινε σε home pose"""
        rospy.loginfo("🏠 Moving to HOME pose...")
        self.bot.arm.go_to_home_pose()
        rospy.sleep(2)
        rospy.loginfo("✓ At HOME\n")
        
    def calculate_grasp_poses(self, target_x, target_y, target_z):
        """Υπολογισμός grasp poses"""
        poses = {}
        poses['approach'] = {
            'x': target_x,
            'y': target_y,
            'z': target_z + self.grasp_config['approach_height']
        }
        poses['grasp'] = {
            'x': target_x,
            'y': target_y,
            'z': target_z + self.grasp_config['grasp_z_offset']
        }
        poses['retreat'] = {
            'x': target_x,
            'y': target_y,
            'z': target_z + self.grasp_config['retreat_height']
        }
        return poses
        
    def move_to_position(self, x, y, z, moving_time=2.0):
        """Move end-effector to position"""
        if not self.validate_position(x, y, z):
            return False
            
        try:
            self.bot.arm.set_ee_pose_components(
                x=x, y=y, z=z,
                moving_time=moving_time,
                accel_time=0.3
            )
            rospy.sleep(moving_time + 0.5)
            return True
        except Exception as e:
            rospy.logerr(f"❌ Motion failed: {e}")
            return False
            
    def pick_object(self, obj_x, obj_y, obj_z):
        """Pick sequence"""
        rospy.loginfo("🤖 PICK SEQUENCE")
        
        if not self.validate_position(obj_x, obj_y, obj_z):
            return False
            
        poses = self.calculate_grasp_poses(obj_x, obj_y, obj_z)
        
        try:
            # Open gripper
            rospy.loginfo("  ↔️  Opening gripper...")
            self.bot.gripper.open()
            rospy.sleep(0.5)
            
            # Approach
            rospy.loginfo(f"  ⬇️  Approaching... (z={poses['approach']['z']:.3f})")
            if not self.move_to_position(
                poses['approach']['x'],
                poses['approach']['y'],
                poses['approach']['z'],
                moving_time=2.0
            ):
                return False
                
            # Grasp
            rospy.loginfo(f"  ⬇️  Descending to grasp... (z={poses['grasp']['z']:.3f})")
            if not self.move_to_position(
                poses['grasp']['x'],
                poses['grasp']['y'],
                poses['grasp']['z'],
                moving_time=1.5
            ):
                return False
                
            # Close gripper
            rospy.loginfo("  🤏 Grasping...")
            self.bot.gripper.close()
            rospy.sleep(1.0)
            
            # Retreat
            rospy.loginfo(f"  ⬆️  Lifting... (z={poses['retreat']['z']:.3f})")
            if not self.move_to_position(
                poses['retreat']['x'],
                poses['retreat']['y'],
                poses['retreat']['z'],
                moving_time=2.0
            ):
                return False
                
            rospy.loginfo("  ✅ PICK complete!\n")
            return True
            
        except Exception as e:
            rospy.logerr(f"❌ Pick failed: {e}")
            return False
            
    def place_object(self, place_x, place_y, place_z):
        """Place sequence"""
        rospy.loginfo("🤖 PLACE SEQUENCE")
        
        if not self.validate_position(place_x, place_y, place_z):
            return False
            
        poses = self.calculate_grasp_poses(place_x, place_y, place_z)
        
        try:
            # Approach
            rospy.loginfo(f"  ➡️  Moving to drop zone... (z={poses['approach']['z']:.3f})")
            if not self.move_to_position(
                poses['approach']['x'],
                poses['approach']['y'],
                poses['approach']['z'],
                moving_time=2.0
            ):
                return False
                
            # Place
            rospy.loginfo(f"  ⬇️  Lowering... (z={poses['grasp']['z']:.3f})")
            if not self.move_to_position(
                poses['grasp']['x'],
                poses['grasp']['y'],
                poses['grasp']['z'],
                moving_time=1.5
            ):
                return False
                
            # Release
            rospy.loginfo("  ↔️  Releasing...")
            self.bot.gripper.open()
            rospy.sleep(1.0)
            
            # Retreat
            rospy.loginfo(f"  ⬆️  Retracting... (z={poses['retreat']['z']:.3f})")
            if not self.move_to_position(
                poses['retreat']['x'],
                poses['retreat']['y'],
                poses['retreat']['z'],
                moving_time=2.0
            ):
                return False
                
            rospy.loginfo("  ✅ PLACE complete!\n")
            return True
            
        except Exception as e:
            rospy.logerr(f"❌ Place failed: {e}")
            return False
            
    def execute_pick_and_place(self, object_coords):
        """Complete workflow"""
        self.is_busy = True
        
        rospy.loginfo("\n" + "🚀 "*25)
        rospy.loginfo("EXECUTING PICK-AND-PLACE WORKFLOW")
        rospy.loginfo("🚀 "*25 + "\n")
        
        try:
            # Pick
            success = self.pick_object(
                object_coords['x'],
                object_coords['y'],
                object_coords['z']
            )
            
            if not success:
                rospy.logerr("❌ Pick failed! Aborting...")
                self.go_to_home_pose()
                self.is_busy = False
                return False
                
            # Place
            success = self.place_object(
                self.drop_location['x'],
                self.drop_location['y'],
                self.drop_location['z']
            )
            
            if not success:
                rospy.logerr("❌ Place failed!")
                self.go_to_home_pose()
                self.is_busy = False
                return False
                
            # Return home
            self.go_to_home_pose()
            
            self.objects_picked += 1
            
            rospy.loginfo("🎉 "*25)
            rospy.loginfo(f"✅✅✅ WORKFLOW COMPLETE! Total objects: {self.objects_picked}")
            rospy.loginfo("🎉 "*25 + "\n")
            rospy.loginfo("Waiting for next object...\n")
            
            self.is_busy = False
            return True
            
        except Exception as e:
            rospy.logerr(f"❌ Workflow error: {e}")
            self.go_to_home_pose()
            self.is_busy = False
            return False
            
    def shutdown(self):
        """Cleanup"""
        rospy.loginfo("\n🛑 Shutting down...")
        self.bot.arm.go_to_sleep_pose()
        rospy.sleep(2)


def main():
    """Main function"""
    try:
        controller = AutomatedPickPlace()
        
        rospy.loginfo("✅ System ready and listening for objects!")
        rospy.loginfo("Press Ctrl+C to stop\n")
        
        rospy.spin()
        
    except KeyboardInterrupt:
        rospy.loginfo("\n⚠️  Keyboard interrupt received")
    except Exception as e:
        rospy.logerr(f"❌ Error: {e}")
    finally:
        if 'controller' in locals():
            controller.shutdown()


if __name__ == '__main__':
    main()