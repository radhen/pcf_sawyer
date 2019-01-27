#! /usr/bin/env python

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

def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)

    parser.add_argument(
        "-ras", "--rot_angle_steps", type=float,
        nargs='+',default=2, help="Number of rotations on one side")
    parser.add_argument(
        "-rass", "--rot_angle_step_size", type=float,
        nargs='+', default=np.deg2rad(20), help="Number of rotations on one side")
    parser.add_argument(
        "-q", "--joint_angles", type=float,
        nargs='+',
        default=[0.158984375, 0.665759765625, -1.53172265625, 1.0492724609375, 0.8098212890625, -1.0504248046875, 2.89727734375],
        help="A list of joint angles, one for each of the 7 joints, J0...J6")
    parser.add_argument(
        "-sr", "--speed_ratio", type=float, default=0.001,
        help="A value between 0.001 (slow) and 1.0 (maximum joint velocity)")
    parser.add_argument(
        "-a", "--accel_ratio", type=float, default=0.001,
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
        "-ef",  "--in_endpoint_frame", action='store_true', default=False,
        help="Set the desired reference frame to endpoint frame; otherwise, it is base frame by default")
    parser.add_argument(
        "-en",  "--endpoint_name", type=str, default='right_gripper',
        help="Set the desired endpoint frame by its name; otherwise, it is right_hand frame by default")
    parser.add_argument(
        "-f", "--force_command", type=float,
        nargs='+', default=[0.0, 0.0, -15.0, 0.0, 0.0, 0.0],
        help="A list of desired force commands, one for each of the 6 directions -- in force control mode this is the vector of desired forces/torques to be regulated in (N) and (Nm), in impedance with force limit mode this vector specifies the magnitude of forces/torques (N and Nm) that the command will not exceed")
    parser.add_argument(
        "-kn", "--K_nullspace", type=float,
        nargs='+', default=[1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0],
        help="A list of desired nullspace stiffnesses, one for each of the 7 joints (a single value can be provided to apply the same value to all the directions) -- units are in (Nm/rad)")
    parser.add_argument(
        "-dd",  "--disable_damping_in_force_control", action='store_true', default=False,
        help="Disable damping in force control")
    parser.add_argument(
        "-dr",  "--disable_reference_resetting", action='store_true', default=False,
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


    limb = Limb()
    traj = MotionTrajectory(limb=limb)
    wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=args.speed_ratio,
                                     max_joint_accel=args.accel_ratio)
    waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)

    # xyz = [0.8, 0.0 , 0.2]
    # des_pose = get_pose(xyz[0], xyz[1], xyz[2], Orientations.FORWARD_POINT)
    # curr_pos = limb.joint_angles()  # Measure current position
    # joint_positions = get_joint_angles(des_pose, limb.name, curr_pos, use_advanced_options=True)  # gets joint positions
    # args.joint_angles = joint_positions.values()
    # args.joint_angles = args.joint_angles.reverse()

    # rotate wrist joint 20deg. ()0.349 rad)
    args.joint_angles[6] = args.joint_angles[6] - (args.rot_angle_step_size * args.rot_angle_steps)

    waypoint.set_joint_angles(joint_angles=args.joint_angles)
    traj.append_waypoint(waypoint.to_msg())

    result = traj.send_trajectory(timeout=args.timeout)

    if result is None:
        rospy.logerr('Trajectory FAILED to send!')
        return
    if result.result:
        rospy.loginfo('Motion controller successfully finished the trajectory with interaction options set!')
    else:
        rospy.logerr('Motion controller failed to complete the trajectory with error %s', result.errorId)

    traj.clear_waypoints()

    ic_pub = InteractionPublisher()

    gd = GetData()
    gd.start_recording()

    for _ in range(2 * args.rot_angle_steps + 1):

        for _ in range(5):
            # print the resultant interaction options once
            rospy.loginfo(interaction_options.to_msg())
            ic_pub = InteractionPublisher()
            rospy.sleep(0.5)
            ic_pub.send_command(interaction_options, args.rate)
            if args.rate == 0:
                rospy.sleep(1)
            rospy.on_shutdown(ic_pub.send_position_mode_cmd)
            args.force_command[2] -= 2
            interaction_options.set_force_command(args.force_command)

        for _ in range(5):
            # print the resultant interaction options once
            rospy.loginfo(interaction_options.to_msg())
            ic_pub = InteractionPublisher()
            rospy.sleep(0.5)
            ic_pub.send_command(interaction_options, args.rate)
            if args.rate == 0:
                rospy.sleep(1)
            rospy.on_shutdown(ic_pub.send_position_mode_cmd)
            args.force_command[2] += 2
            interaction_options.set_force_command(args.force_command)

        args.joint_angles[6] = args.joint_angles[6] + args.rot_angle_step_size
        waypoint.set_joint_angles(joint_angles=args.joint_angles)
        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory(timeout=args.timeout)

        if result is None:
            rospy.logerr('Trajectory FAILED to send!')
        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory with interaction options set!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)

        traj.clear_waypoints()
        args.force_command[2] = -15

    # Finally robot go to home position
    waypoint.set_joint_angles(joint_angles=[0.158984375, 0.665759765625, -1.53172265625, 1.0492724609375, 0.8098212890625, -1.0504248046875, 2.89727734375])
    traj.append_waypoint(waypoint.to_msg())

    result = traj.send_trajectory(timeout=args.timeout)
    if result is None:
        rospy.logerr('Trajectory FAILED to send!')
        return
    if result.result:
        rospy.loginfo('Motion controller successfully finished the trajectory with interaction options set!')
    else:
        rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                     result.errorId)


    gd.stop_recording()
    gd.convertandsave('motion3_test_4_fs')

    print ("Done. Saved data.")


if __name__ == '__main__':
    main()
