#!/usr/bin/env python3

import rospy
import time
from widowxl_moveit_config_oj.msg import ObjectPose
from interbotix_xs_modules.arm import InterbotixManipulatorXS


class PickAndPlaceNode:
    def __init__(self):
        # rospy.init_node('pick_and_place_node', anonymous=True)

        self.bot = InterbotixManipulatorXS("widowxl", "arm", "gripper")
        self.bot.gripper.set_pressure(1)
        self.bot.arm.go_to_home_pose()
        self.bot.gripper.open()

        self.is_busy = False  # αποτρέπει διπλό trigger αν έρθουν πολλά msg

        rospy.Subscriber('/obj_pose', ObjectPose, self.callback)
        rospy.loginfo("Pick and Place Node ready — listening on /obj_pose")
        rospy.spin()

    def callback(self, msg: ObjectPose):
        if self.is_busy:
            rospy.logwarn("Still executing previous pick & place — skipping msg")
            return

        self.is_busy = True
        rospy.loginfo(
            f"Received pose → x:{msg.x:.4f}, y:{msg.y:.4f}, z:{msg.z:.4f} | id:'{msg.object_id}'"
        )

        try:
            self.pick_and_place(msg.x, msg.y, msg.z)
        except Exception as e:
            rospy.logerr(f"Pick & place failed: {e}")
        finally:
            self.is_busy = False

    def pick_and_place(self, x, y, z):
        bot = self.bot

        # Approach από πάνω
        bot.arm.set_ee_pose_components(x=x, y=y, z=z + 0.05, pitch=1.5)

        # Κατεβαίνει στο αντικείμενο
        bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=1.5)
        bot.gripper.close()

        time.sleep(2)

        # Ανεβαίνει με το αντικείμενο
        bot.arm.set_ee_pose_components(x=x, y=y, z=z + 0.1, pitch=1.5)

        # Επιστροφή στη home
        bot.arm.go_to_home_pose()
        bot.gripper.open()

        rospy.loginfo("Pick & place completed successfully")


if __name__ == '__main__':
    PickAndPlaceNode()
