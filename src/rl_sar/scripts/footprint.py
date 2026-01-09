#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import TransformStamped
import math

if __name__ == '__main__':
    rospy.init_node('base_footprint_broadcaster')

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                "odom", "trunk", rospy.Time(0)
            )

            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)

            br.sendTransform(
                (trans[0], trans[1], 0.0),  # z 强制为 0
                tf.transformations.quaternion_from_euler(0, 0, yaw),
                rospy.Time.now(),
                "base_footprint",
                "odom"
            )

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        rate.sleep()
