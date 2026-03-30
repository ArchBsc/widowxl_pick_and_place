#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def color_publisher():
    rospy.init_node('color_publisher', anonymous=True)
    pub = rospy.Publisher('/color_cube_topic', String, queue_size=10)
    rate = rospy.Rate(2)  # Publish every 2 seconds

    colors = ["Red", "Green", "Blue"]
    index = 0

    while not rospy.is_shutdown():
        color = colors[index % len(colors)]
        rospy.loginfo("Publishing color: %s" % color)
        pub.publish(color)
        index += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        color_publisher()
    except rospy.ROSInterruptException:
        pass

