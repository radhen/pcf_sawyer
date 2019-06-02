#! /usr/bin/env python

'''
This scripts does the quality control testing. Basically it makes the Sawyer robot poke the sensor multiple times
from the normal direction. Video is "My Drive/Cleveland Contract/Videos" on the Google Drive named quality_control_testing.mp4
'''

import rospy
import argparse
from geometry_msgs.msg import Pose
from intera_core_msgs.msg import InteractionControlCommand
from intera_motion_interface import (InteractionOptions, InteractionPublisher)
from intera_motion_interface.utility_functions import int2bool

from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions,
    InteractionOptions
)
from intera_interface import Limb
from get_data import GetData

from grip_and_record.inverse_kin import *
from grip_and_record.robot_utils import Orientations
import numpy as np

import intera_interface
from intera_interface import CHECK_VERSION
from tf.transformations import euler_from_quaternion, quaternion_from_euler



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
    limb.set_joint_position_speed(0.06)
    return limb


def move_arm(limb, xyz, roll_angle, pitch_angle):
    q_1 = quaternion_from_euler(np.deg2rad(roll_angle), 0.0, 0.0)
    q_2 = quaternion_from_euler(0.0, np.deg2rad(pitch_angle), 0.0)
    scalar = (q_1[3] * q_2[3]) - np.dot(np.array([q_1[0], q_1[1], q_1[2]]), np.array([q_2[0], q_2[1], q_2[2]]))
    vector = np.multiply(np.array([q_1[0], q_1[1], q_1[2]]), q_2[3]) + np.multiply(np.array([q_2[0], q_2[1], q_2[2]]),
                                                                                   q_1[3]) + np.cross(
        np.array([q_2[0], q_2[1], q_2[2]]), np.array([q_1[0], q_1[1], q_1[2]]))
    q_3 = [vector[0], vector[1], vector[2], scalar]
    orientation = Quaternion(x=q_3[0], y=q_3[1], z=q_3[2], w=q_3[3])

    des_pose = get_pose(xyz[0], xyz[1], xyz[2], orientation)
    curr_pose = limb.joint_angles()  # Measure current position
    joint_positions = get_joint_angles(des_pose, limb.name, curr_pose, use_advanced_options=True)
    limb.move_to_joint_positions(joint_positions, timeout=20, threshold=0.01)  # Send the command to the arm


def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)

    parser.add_argument(
        "-rolls", "--roll_steps", type=float,
        nargs='+',default=0, help="Number of rotations on one side")
    parser.add_argument(
        "-rollss", "--roll_step_size", type=float,
        nargs='+', default=0, help="Size of roll in degree")
    parser.add_argument(
        "-pitchs", "--pitch_steps", type=float,
        nargs='+', default=0, help="Number of rotations on one side")
    parser.add_argument(
        "-pitchss", "--pitch_step_size", type=float,
        nargs='+', default=0, help="Size of pitch in degree")
    parser.add_argument(
        "-q", "--joint_angles", type=float,
        nargs='+',
        default=[0.158984375, 0.665759765625, -1.53172265625, 1.0492724609375, 0.8098212890625, -1.0504248046875, 2.89727734375],
        help="A list of joint angles, one for each of the 7 joints, J0...J6")
    parser.add_argument(
        "-sr", "--speed_ratio", type=float, default=0.06,
        help="A value between 0.001 (slow) and 1.0 (maximum joint velocity)")
    parser.add_argument(
        "-a", "--accel_ratio", type=float, default=0.06,
        help="A value between 0.001 (slow) and 1.0 (maximum joint accel)")
    parser.add_argument(
        "-s",  "--interaction_active", type=int, default=1, choices = [0, 1],
        help="Activate (1) or Deactivate (0) interaction controller")
    parser.add_argument(
        "-k", "--K_impedance", type=float,
        nargs='+', default=[1300.0, 1300.0, 1300.0, 30.0, 30.0, 30.0],
        help="A list of desired stiffnesses, one for each of the 6 directions -- stiffness units are (N/m) for first 3 and (Nm/rad) for second 3 values")
    parser.add_argument(
        "-m", "--max_impedance", type=int,
        nargs='+', default=[0, 0, 1, 0, 0, 0], choices = [0, 1],
        help="A list of maximum stiffness behavior state, one for each of the 6 directions (a single value can be provided to apply the same value to all the directions) -- 0 for False, 1 for True")
    parser.add_argument(
        "-md", "--interaction_control_mode", type=int,
        nargs='+', default=[1, 1, 2, 1, 1, 1], choices = [1,2,3,4],
        help="A list of desired interaction control mode (1: impedance, 2: force, 3: impedance with force limit, 4: force with motion limit), one for each of the 6 directions")
    parser.add_argument(
        "-fr", "--interaction_frame", type=float,
        nargs='+', default=[0, 0, 0, 1, 0, 0, 0],
        help="Specify the reference frame for the interaction controller -- first 3 values are positions [m] and last 4 values are orientation in quaternion (w, x, y, z) which has to be normalized values")
    parser.add_argument(
        "-ef",  "--in_endpoint_frame", action='store_true', default=True,
        help="Set the desired reference frame to endpoint frame; otherwise, it is base frame by default")
    parser.add_argument(
        "-en",  "--endpoint_name", type=str, default='right_gripper',
        help="Set the desired endpoint frame by its name; otherwise, it is right_hand frame by default")
    parser.add_argument(
        "-f", "--force_command", type=float,
        nargs='+', default=[0.0, 0.0, 7.0, 0.0, 0.0, 0.0],
        help="A list of desired force commands, one for each of the 6 directions -- in force control mode this is the vector of desired forces/torques to be regulated in (N) and (Nm), in impedance with force limit mode this vector specifies the magnitude of forces/torques (N and Nm) that the command will not exceed")
    parser.add_argument(
        "-kn", "--K_nullspace", type=float,
        nargs='+', default=[1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0],
        help="A list of desired nullspace stiffnesses, one for each of the 7 joints (a single value can be provided to apply the same value to all the directions) -- units are in (Nm/rad)")
    parser.add_argument(
        "-dd",  "--disable_damping_in_force_control", action='store_true', default=False,
        help="Disable damping in force control")
    parser.add_argument(
        "-dr",  "--disable_reference_resetting", action='store_true', default=True,
        help="The reference signal is reset to actual position to avoid jerks/jumps when interaction parameters are changed. This option allows the user to disable this feature.")
    parser.add_argument(
        "-rc",  "--rotations_for_constrained_zeroG", action='store_true', default=False,
        help="Allow arbitrary rotational displacements from the current orientation for constrained zero-G (works only with a stationary reference orientation)")
    parser.add_argument(
        "-r",  "--rate", type=int, default=0,
        help="A desired publish rate for updating interaction control commands (10Hz by default) -- a rate 0 publish once and exits which can cause the arm to remain in interaction control.")
    parser.add_argument(
        "--timeout", type=float, default=None,
        help="Max time in seconds to complete motion goal before returning. None is interpreted as an infinite timeout.")


    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('set_interaction_options')

    # set the interaction control options in the current configuration
    interaction_options = InteractionOptions()

    interaction_options.set_interaction_control_active(int2bool(args.interaction_active))
    interaction_options.set_K_impedance(args.K_impedance)
    interaction_options.set_max_impedance(int2bool(args.max_impedance))
    interaction_options.set_interaction_control_mode(args.interaction_control_mode)
    interaction_options.set_in_endpoint_frame(args.in_endpoint_frame)
    interaction_options.set_force_command(args.force_command)
    interaction_options.set_K_nullspace(args.K_nullspace)

    if len(args.interaction_frame) == 7:
        interaction_frame = Pose()
        interaction_frame.position.x = args.interaction_frame[0]
        interaction_frame.position.y = args.interaction_frame[1]
        interaction_frame.position.z = args.interaction_frame[2]
        interaction_frame.orientation.w = args.interaction_frame[3]
        interaction_frame.orientation.x = args.interaction_frame[4]
        interaction_frame.orientation.y = args.interaction_frame[5]
        interaction_frame.orientation.z = args.interaction_frame[6]
        interaction_options.set_interaction_frame(interaction_frame)
    else:
        rospy.logerr('Invalid input to interaction_frame. Must be 7 elements.')

    interaction_options.set_disable_damping_in_force_control(args.disable_damping_in_force_control)
    interaction_options.set_disable_reference_resetting(args.disable_reference_resetting)
    interaction_options.set_rotations_for_constrained_zeroG(args.rotations_for_constrained_zeroG)

    limb_name = "right"
    limb = init_robot(limb_name=limb_name)

    xyz = [0.7, 0.0, 0.13]

    des_pose = get_pose(xyz[0], xyz[1], xyz[2], Orientations.DOWNWARD_POINT)
    curr_pose = limb.joint_angles()  # Measure current position
    joint_positions = get_joint_angles(des_pose, limb.name, curr_pose, use_advanced_options=True)
    limb.move_to_joint_positions(joint_positions, timeout=20, threshold=0.01)  # Send the command to the arm

    roll_angle = args.roll_step_size * args.roll_steps
    pitch_angle = 180 - (args.pitch_step_size * args.pitch_steps)

    move_arm(limb, xyz, roll_angle, pitch_angle)

    ic_pub = InteractionPublisher()


    for j in range(2 * args.pitch_steps + 1):

        for i in range(2 * args.roll_steps + 1):

            gd = GetData()
            gd.start_recording()

            for k in range(100):
                print k

                move_arm(limb, [0.7,0.0,0.12], roll_angle, pitch_angle)

                for _ in range(10):
                    # print the resultant interaction options once
                    # rospy.loginfo(interaction_options.to_msg())
                    ic_pub.send_command(interaction_options, args.rate)
                    if args.rate == 0:
                        rospy.sleep(0.1)
                    args.force_command[2] += 3
                    interaction_options.set_force_command(args.force_command)

                for _ in range(10):
                    # print the resultant interaction options once
                    # rospy.loginfo(interaction_options.to_msg())
                    ic_pub.send_command(interaction_options, args.rate)
                    if args.rate == 0:
                        rospy.sleep(0.1)
                    args.force_command[2] -= 3
                    interaction_options.set_force_command(args.force_command)


                ic_pub.send_position_mode_cmd()

                move_arm(limb, xyz, roll_angle, pitch_angle)

            gd.stop_recording()
            gd.convertandsave(i, j, k)

            roll_angle -= args.roll_step_size
            move_arm(limb, xyz, roll_angle, pitch_angle)

            args.force_command[2] = 6

        # reset roll angle
        roll_angle = args.roll_step_size * args.roll_steps
        # increment the pitch angle
        pitch_angle += args.pitch_step_size
        move_arm(limb, xyz, roll_angle, pitch_angle)

    print ("Done. Saved data.")


if __name__ == '__main__':
    main()
