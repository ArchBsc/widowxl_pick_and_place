#!/usr/bin/env python3

"""
Mock Camera Node - Προσομοίωση Intel RealSense D435

Δίνει fake object detections για testing χωρίς πραγματική camera.
Στο μέλλον θα αντικατασταθεί με πραγματική RealSense integration.

Usage:
    python3 widowxl_mock_camera.py
    
Ή σε άλλο terminal για integration:
    # Terminal 1
    roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=widowxl
    
    # Terminal 2
    python3 widowxl_mock_camera.py
    
    # Terminal 3
    python3 widowxl_pick_place_auto.py  # (το φτιάχνουμε παρακάτω)
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
import numpy as np


class MockRealSenseCamera:
    def __init__(self):
        rospy.init_node('mock_realsense_camera', anonymous=True)
        
        # Publishers
        self.object_pose_pub = rospy.Publisher(
            '/widowxl/detected_object',
            PoseStamped,
            queue_size=10
        )
        
        self.detection_status_pub = rospy.Publisher(
            '/widowxl/detection_status',
            String,
            queue_size=10
        )
        
        self.object_detected_pub = rospy.Publisher(
            '/widowxl/object_detected',
            Bool,
            queue_size=10
        )
        
        # Test objects database
        # Αυτά προσομοιώνουν detections από τη camera
        self.test_objects = [
            {
                'name': 'bottle_front',
                'x': 0.25,      # 25cm μπροστά
                'y': 0.08,      # 8cm δεξιά
                'z': 0.025,     # 2.5cm ύψος
                'type': 'bottle'
            },
            {
                'name': 'bottle_left',
                'x': 0.23,
                'y': -0.10,     # 10cm αριστερά
                'z': 0.030,
                'type': 'bottle'
            },
            {
                'name': 'box_center',
                'x': 0.28,
                'y': 0.0,       # Κέντρο
                'z': 0.015,
                'type': 'box'
            },
            {
                'name': 'can_right',
                'x': 0.20,
                'y': 0.15,      # 15cm δεξιά
                'z': 0.035,
                'type': 'can'
            },
        ]
        
        self.current_object_idx = 0
        
        # Camera to base transform (ΑΛΛΑΞΕ με τις πραγματικές τιμές σου)
        self.camera_offset = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.30    # Π.χ. 30cm ψηλά από base
        }
        
        rospy.loginfo("="*60)
        rospy.loginfo("Mock RealSense Camera Node Started")
        rospy.loginfo(f"Loaded {len(self.test_objects)} test objects:")
        for obj in self.test_objects:
            rospy.loginfo(f"  - {obj['name']}: ({obj['x']:.3f}, {obj['y']:.3f}, {obj['z']:.3f})")
        rospy.loginfo("="*60)
        
    def add_noise(self, x, y, z, noise_std=0.003):
        """
        Προσθήκη Gaussian noise για realism
        Προσομοιώνει το measurement noise της camera
        """
        x_noisy = x + np.random.normal(0, noise_std)
        y_noisy = y + np.random.normal(0, noise_std)
        z_noisy = z + np.random.normal(0, noise_std * 0.5)  # Λιγότερο noise στο z
        return x_noisy, y_noisy, z_noisy
        
    def get_next_detection(self, add_noise=True):
        """
        Επιστρέφει το επόμενο test object
        
        Returns:
            dict με object info ή None αν δεν υπάρχουν άλλα
        """
        if self.current_object_idx >= len(self.test_objects):
            rospy.logwarn("No more test objects. Resetting to beginning.")
            self.current_object_idx = 0
            return None
            
        obj = self.test_objects[self.current_object_idx].copy()
        self.current_object_idx += 1
        
        # Add noise αν θέλουμε realism
        if add_noise:
            obj['x'], obj['y'], obj['z'] = self.add_noise(obj['x'], obj['y'], obj['z'])
            
        return obj
        
    def publish_detection(self, obj):
        """
        Publish detected object pose
        
        Args:
            obj: dict με 'x', 'y', 'z', 'name', 'type'
        """
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"  # ΑΛΛΑΞΕ αν χρειάζεται
        
        # Position
        pose_msg.pose.position.x = obj['x']
        pose_msg.pose.position.y = obj['y']
        pose_msg.pose.position.z = obj['z']
        
        # Orientation (για top-down grasp, δεν χρειάζεται rotation)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        # Publish
        self.object_pose_pub.publish(pose_msg)
        self.object_detected_pub.publish(True)
        
        # Status message
        status_msg = (f"[DETECTION] {obj['name']} ({obj['type']}) at "
                     f"x={obj['x']:.3f}m, y={obj['y']:.3f}m, z={obj['z']:.3f}m")
        self.detection_status_pub.publish(status_msg)
        
        rospy.loginfo(status_msg)
        
        return pose_msg
        
    def run_continuous_detection(self, rate_hz=0.5, add_noise=True):
        """
        Continuous detection loop
        Δημοσιεύει ένα object κάθε 1/rate_hz seconds
        
        Args:
            rate_hz: Συχνότητα detections σε Hz (default: 0.5 = κάθε 2 sec)
            add_noise: Προσθήκη noise στα measurements
        """
        rate = rospy.Rate(rate_hz)
        
        rospy.loginfo(f"\nStarting continuous detection at {rate_hz} Hz")
        rospy.loginfo("Press Ctrl+C to stop\n")
        
        while not rospy.is_shutdown():
            obj = self.get_next_detection(add_noise=add_noise)
            
            if obj is None:
                rospy.loginfo("Completed one cycle. Waiting 3 seconds...")
                rospy.sleep(3)
                continue
                
            self.publish_detection(obj)
            rate.sleep()
            
    def run_manual_mode(self):
        """
        Manual mode - publish detections on demand
        """
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("MANUAL MODE")
        rospy.loginfo("="*60)
        rospy.loginfo("Press Enter to publish next detection, 'q' to quit\n")
        
        while not rospy.is_shutdown():
            try:
                user_input = input("Press Enter for next object (or 'q' to quit): ")
                
                if user_input.lower() == 'q':
                    break
                    
                obj = self.get_next_detection(add_noise=True)
                
                if obj is None:
                    self.current_object_idx = 0  # Reset
                    rospy.loginfo("Reset to first object\n")
                    continue
                    
                self.publish_detection(obj)
                rospy.loginfo("")
                
            except KeyboardInterrupt:
                break
                
        rospy.loginfo("Exiting manual mode")


def main():
    """Main function"""
    
    print("\n" + "="*70)
    print("WidowXL Mock Camera - RealSense D435 Simulator")
    print("="*70)
    print("\nModes:")
    print("  1. Continuous - Auto-publish detections")
    print("  2. Manual     - Press Enter to publish")
    print("  3. ROS only   - Just run node (για integration με άλλα scripts)")
    print("="*70)
    
    try:
        camera = MockRealSenseCamera()
        
        # Επιλογή mode
        mode = input("\nSelect mode (1/2/3): ").strip()
        
        if mode == '1':
            # Continuous mode
            rate = float(input("Detection rate in Hz (default 0.5): ") or "0.5")
            camera.run_continuous_detection(rate_hz=rate, add_noise=True)
            
        elif mode == '2':
            # Manual mode
            camera.run_manual_mode()
            
        else:
            # ROS node mode
            rospy.loginfo("\nRunning as ROS node. Waiting for requests...")
            rospy.loginfo("Call service or subscribe to topics to get detections\n")
            rospy.spin()
            
    except KeyboardInterrupt:
        rospy.loginfo("\nShutting down camera node")
    except Exception as e:
        rospy.logerr(f"Error: {e}")


if __name__ == '__main__':
    main()