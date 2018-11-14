#!/usr/bin/env python

import rospy
import tf


def BroadcastFrame():
    broadcast = tf.TransformBroadcaster()
    rate = rospy.Rate(1000)
    start1 = rospy.get_time()
    while not rospy.is_shutdown():# and (rospy.get_time() - start1 < 60.0):
        broadcast.sendTransform([0, 0, 0.2], [0,0,0,1],
                              rospy.Time.now(),
                              "sensor_frame",
                              "right_wrist")
        rate.sleep()
    print ("Finished publishing DEBUG gripper finger frame")

if __name__ == '__main__':
    rospy.init_node('publish_sensor_frame_node')
    broadcast = tf.TransformBroadcaster()
    BroadcastFrame()
