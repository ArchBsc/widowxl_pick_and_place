#!/usr/bin/env python3

import rospy
from widowxl_moveit_config_oj.msg import ObjectPose


def main():
    rospy.init_node('object_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/obj_pose', ObjectPose, queue_size=10)

    rospy.sleep(1.0)  # περιμένουμε να ανέβει το topic

    msg = ObjectPose()
    msg.x = 0.20
    msg.y = -0.05
    msg.z = -0.0605
    msg.object_id = "test_object"

    rospy.loginfo(f"Publishing pose → x:{msg.x}, y:{msg.y}, z:{msg.z}")
    pub.publish(msg)

    rospy.sleep(0.5)


if __name__ == '__main__':
    main()
