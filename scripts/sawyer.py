#!/usr/bin/env python

import rospy
import intera_interface
from intera_interface import CHECK_VERSION
# from intera_interface import (
#     Gripper,
#     Lights,
#     Cuff,
#     RobotParams,
# )

import grip_and_record.inverse_kin
from grip_and_record.robot_utils import Orientations

import numpy as np
import time

from get_data import GetData

from std_msgs.msg import Float32MultiArray

from intera_core_msgs.msg import EndpointState


def init_robot(limb_name):
    epilog = """
    See help inside the example with the '?' key for key bindings.
        """
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return

    rp.log_message('Initializing node... ')
    rospy.init_node("move_and_grip")

    rp.log_message('Getting robot state...  ')
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")
        if not init_state:
            rp.log_message('Disabling robot...')
            rs.disable()

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()
    if not limb_name in valid_limbs:
        rp.log_message(("Right is not a valid limb on this robot. "
                        "Exiting."), "ERROR")
        return
    limb = intera_interface.Limb(limb_name)
    limb.set_joint_position_speed(0.05)
    # Move to a safe position
    # goto_rest_pos(limb=limb)

    return limb


def goto_rest_pos(limb, verbosity=1):
    """
    Move the arm to a safe rest position
    :param limb: link to the limb being used
    :param blocking: Bool. is it a blocking operation? (ie., do we wait until the end of the operation?)
    :param verbosity: verbosity level. >0 print stuff
    :return:
    """
    xyz_rest = [0.7, 0.0, 0.15]
    if verbosity > 0:
        rp = intera_interface.RobotParams()  # For logging
        rp.log_message('Moving to rest position')
    goto_EE_xyz(limb=limb, xyz=xyz_rest, orientation=Orientations.FORWARD_POINT, verbosity=verbosity - 1,
                rest_pos=True)


def goto_EE_xyz(limb, xyz, orientation=Orientations.DOWNWARD_ROTATED, verbosity=1, rest_pos=False):
    """
    Move the End-effector to the desired XYZ position and orientation, using inverse kinematic
    :param limb: link to the limb being used
    :param xyz: list or array [x,y,z] with the coordinates in XYZ positions in limb reference frame
    :param orientation:
    :param verbosity: verbosity level. >0 print stuff
    :return:
    """
    try:
        if verbosity > 0:
            rp = intera_interface.RobotParams()  # For logging
            rp.log_message('Moving to x=%f y=%f z=%f' % (xyz[0], xyz[1], xyz[2]))
        if not rest_pos:
            # Make sure that the XYZ position is valid, and doesn't collide with the cage
            assert (xyz[0] >= bounds_table[0, 0]) and (xyz[0] <= bounds_table[0, 1]), 'X is outside of the bounds'
            assert (xyz[1] >= bounds_table[1, 0]) and (xyz[1] <= bounds_table[1, 1]), 'Y is outside of the bounds'
            assert (xyz[2] >= lower_bound_z), 'Z is outside of the bounds'
        des_pose = grip_and_record.inverse_kin.get_pose(xyz[0], xyz[1], xyz[2], orientation)
        curr_pos = limb.joint_angles()  # Measure current position
        joint_positions = grip_and_record.inverse_kin.get_joint_angles(des_pose, limb.name, curr_pos,
                                                                       use_advanced_options=True)  # gets joint positions
        limb.move_to_joint_positions(joint_positions, timeout=20, threshold=0.01)  # Send the command to the arm
        # print("done moving")
    except UnboundLocalError:
        pose_dict = limb.endpoint_pose()
        pose_pos = pose_dict['position']
        current_xyz = [pose_pos.x, pose_pos.y, pose_pos.z]
        halfway_xyz = ((np.array(xyz) + np.array(current_xyz)) / 2.0).tolist()
        if np.linalg.norm(np.array(current_xyz) - np.array(halfway_xyz)) > 0.00001:
            time.sleep(0.2)
            if rest_pos:
                # goto_EE_xyz(limb, halfway_xyz, orientation, rest_pos=True)
                goto_EE_xyz(limb, xyz, orientation, rest_pos=True)
            else:
                # goto_EE_xyz(limb, halfway_xyz, orientation)
                goto_EE_xyz(limb, xyz, orientation, rest_pos=True)
        else:
            print("WoooOooOW")
            # goto_EE_xyz(limb, [0.55, 0.00, 0.50], orientation, rest_pos=True)
            goto_rest_pos(limb)


def main():
    # Make required initiations
    limb_name = "right"
    limb = init_robot(limb_name=limb_name)

    goto_EE_xyz(limb=limb, xyz=[0.89, 0.0, -0.185], orientation=Orientations.SLIGHT_BACK, rest_pos=True)

    # gd = GetData()
    # gd.start_recording()

    # for _ in range(10):
    #     goto_EE_xyz(limb=limb, xyz=[0.7, 0.0, 0.06], orientation=Orientations.SLIGHT_RIGHT, rest_pos=True)
    #     goto_EE_xyz(limb=limb, xyz=[0.7, 0.0, 0.06], orientation=Orientations.SLIGHT_LEFT , rest_pos=True)
    #
    #     # Neutral position
    #     goto_EE_xyz(limb=limb, xyz=[0.7, 0.0, 0.06], orientation=Orientations.FORWARD_POINT, rest_pos=True)
    #
    #     goto_EE_xyz(limb=limb, xyz=[0.7, 0.0, 0.06], orientation=Orientations.SLIGHT_FRONT, rest_pos=True)
    #     goto_EE_xyz(limb=limb, xyz=[0.7, 0.0, 0.06], orientation=Orientations.SLIGHT_BACK, rest_pos=True)
    #
    #     # Neutral position
    #     goto_EE_xyz(limb=limb, xyz=[0.7, 0.0, 0.06], orientation=Orientations.FORWARD_POINT, rest_pos=True)

    # gd.stop_recording()
    # gd.convertandsave(2)

    # goto_EE_xyz(limb=limb, xyz=[0.7, 0.0, 0.06], orientation=Orientations.FORWARD_POINT, rest_pos=True)


    # IMPEDANCE CONTROL THAT I WISH TO IMPLEMENT
    # https://rethinkrobotics.interaforum.com/topic/569-endpoint-impedance-control-for-sawyer-intera-52/






if __name__ == '__main__':
    main()
