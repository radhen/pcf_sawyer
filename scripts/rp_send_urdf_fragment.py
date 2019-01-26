#!/usr/bin/python

import os
import sys
import argparse

import rospy
try:
    import xacro_jade as xacro
except ImportError:
    import xacro


from intera_core_msgs.msg import (
    URDFConfiguration,
)

class urdf_fragment(object):

    def __init__(self):
        self.link = "right_hand"
        self.joint = "right_gripper_base"
        self.file_path = '/home/radhen/catkin_ws/src/intera_common/intera_tools_description/urdf/pneumatic_gripper/example_end_effector.urdf.xacro'


    def xacro_parse(self, filename):
        doc = xacro.parse(None, filename)
        xacro.process_doc(doc, in_order=True)
        return doc.toprettyxml(indent='  ')

    def send_urdf(self, parent_link, root_joint, urdf_filename):

        msg = URDFConfiguration()
        # The updating the time parameter tells
        # the robot that this is a new configuration.
        # Only update the time when an updated internal
        # model is required. Do not continuously update
        # the time parameter.
        msg.time = rospy.Time.now()
        # link to attach this urdf to onboard the robot
        msg.link = parent_link
        # root linkage in your URDF Fragment
        msg.joint = root_joint
        msg.urdf = self.xacro_parse(urdf_filename)
        pub = rospy.Publisher('/robot/urdf', URDFConfiguration, queue_size=10)
        rate = rospy.Rate(5) # 5hz
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            pub.publish(msg)
            rate.sleep()


def main():
    rospy.init_node('rsdk_configure_urdf', anonymous=True)
    uf = urdf_fragment()
    uf.send_urdf(uf.link, uf.joint, uf.file_path)

if __name__ == '__main__':
    sys.exit(main())
