#! /usr/bin/env python

import rospy
import argparse
from geometry_msgs.msg import Pose
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions,
    InteractionOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from intera_interface import Limb
from intera_motion_interface.utility_functions import int2bool

from get_data import GetData

def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        "-q", "--joint_angles", type=float,
        nargs='+', default=[-0.155232421875, 0.4621865234375, -0.3448271484375, 0.4330361328125, 0.017708984375, -0.946375, 2.040958984375],
        help="A list of joint angles, one for each of the 7 joints, J0...J6")
    parser.add_argument(
        "-s",  "--speed_ratio", type=float, default=0.05,
        help="A value between 0.001 (slow) and 1.0 (maximum joint velocity)")
    parser.add_argument(
        "-a",  "--accel_ratio", type=float, default=0.05,
        help="A value between 0.001 (slow) and 1.0 (maximum joint accel)")
    parser.add_argument(
        "-t", "--trajType", type=str, default='CARTESIAN',
        choices=['JOINT', 'CARTESIAN'],
        help="trajectory interpolation type")
    parser.add_argument(
        "-st",  "--interaction_active", type=int, default=1, choices = [0, 1],
        help="Activate (1) or Deactivate (0) interaction controller")
    parser.add_argument(
        "-k", "--K_impedance", type=float,
        nargs='+', default=[1000.0, 1000.0, 1000.0, 15.0, 15.0, 15.0],
        help="A list of desired stiffnesses, one for each of the 6 directions -- stiffness units are (N/m) for first 3 and (Nm/rad) for second 3 values")
    parser.add_argument(
        "-m", "--max_impedance", type=int,
        nargs='+', default=[0, 0, 1, 0, 0, 0], choices = [0, 1],
        help="A list of impedance modulation state, one for each of the 6 directions (a single value can be provided to apply the same value to all the directions) -- 0 for False, 1 for True")
    parser.add_argument(
        "-md", "--interaction_control_mode", type=int,
        nargs='+', default=[1, 1, 2, 1, 1, 1], choices = [1,2,3,4],
        help="A list of desired interaction control mode (1: impedance, 2: force, 3: impedance with force limit, 4: force with motion limit), one for each of the 6 directions")
    parser.add_argument(
        "-fr", "--interaction_frame", type=float,
        nargs='+', default=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        help="Specify the reference frame for the interaction controller -- first 3 values are positions [m] and last 4 values are orientation in quaternion (w, x, y, z)")
    parser.add_argument(
        "-ef",  "--in_endpoint_frame", action='store_true', default=False,
        help="Set the desired reference frame to endpoint frame; otherwise, it is base frame by default")
    parser.add_argument(
        "-en",  "--endpoint_name", type=str, default='right_hand',
        help="Set the desired endpoint frame by its name; otherwise, it is right_hand frame by default")
    parser.add_argument(
        "-f", "--force_command", type=float,
        nargs='+', default=[0.0, 0.0, -35.0, 0.0, 0.0, 0.0],
        help="A list of desired force commands, one for each of the 6 directions -- in force control mode this is the vector of desired forces/torques to be regulated in (N) and (Nm), in impedance with force limit mode this vector specifies the magnitude of forces/torques (N and Nm) that the command will not exceed")
    parser.add_argument(
        "-kn", "--K_nullspace", type=float,
        nargs='+', default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        help="A list of desired nullspace stiffnesses, one for each of the 7 joints (a single value can be provided to apply the same value to all the directions) -- units are in (Nm/rad)")
    parser.add_argument(
        "-dd",  "--disable_damping_in_force_control", action='store_true', default=False,
        help="Disable damping in force control")
    parser.add_argument(
        "-dr",  "--disable_reference_resetting", action='store_true', default=False,
        help="The reference signal is reset to actual position to avoid jerks/jumps when interaction parameters are changed. This option allows the user to disable this feature.")
    parser.add_argument(
        "-rc",  "--rotations_for_constrained_zeroG", action='store_true', default=False,
        help="Allow arbitrary rotational displacements from the current orientation for constrained zero-G (use only for a stationary reference orientation)")
    parser.add_argument(
        "--timeout", type=float, default=None,
        help="Max time in seconds to complete motion goal before returning. None is interpreted as an infinite timeout.")

    args = parser.parse_args(rospy.myargv()[1:])

    try:
        rospy.init_node('go_to_joint_angles_in_contact_py')
        limb = Limb()
        traj = MotionTrajectory(limb = limb)

        wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=args.speed_ratio,
                                        max_joint_accel=args.accel_ratio)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

        joint_angles = limb.joint_ordered_angles()
        waypoint.set_joint_angles(joint_angles = joint_angles)
        traj.append_waypoint(waypoint.to_msg())

        if len(args.joint_angles) != len(joint_angles):
            rospy.logerr('The number of joint_angles must be %d', len(joint_angles))
            return None

        waypoint.set_joint_angles(joint_angles = args.joint_angles)
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

        for _ in range(5):

            # slight right (30 degree)
            waypoint.set_joint_angles([-0.155232421875, 0.4621865234375, -0.3448271484375, 0.4330361328125, 0.017708984375, -0.946375, 2.56455798437])
            traj.append_waypoint(waypoint.to_msg())

            # neutral position
            waypoint.set_joint_angles([-0.155232421875, 0.4621865234375, -0.3448271484375, 0.4330361328125, 0.017708984375, -0.946375, 2.040958984375])
            traj.append_waypoint(waypoint.to_msg())

            # slight left (30 degree)
            waypoint.set_joint_angles([-0.155232421875, 0.4621865234375, -0.3448271484375, 0.4330361328125, 0.017708984375, -0.946375, 1.51735998438])
            traj.append_waypoint(waypoint.to_msg())

            # neutral position
            waypoint.set_joint_angles([-0.155232421875, 0.4621865234375, -0.3448271484375, 0.4330361328125, 0.017708984375, -0.946375, 2.040958984375])
            traj.append_waypoint(waypoint.to_msg())

            # slight front (30 degree)
            waypoint.set_joint_angles([-0.4005859375, 0.1294775390625, 0.4800302734375, 0.6989287109375, -0.591427734375, -0.216119140625, 1.6886953125])
            traj.append_waypoint(waypoint.to_msg())

            # neutral position
            waypoint.set_joint_angles([-0.155232421875, 0.4621865234375, -0.3448271484375, 0.4330361328125, 0.017708984375, -0.946375, 2.040958984375])
            traj.append_waypoint(waypoint.to_msg())

            # slight back (30 degree)
            waypoint.set_joint_angles([-0.4233544921875, 0.503890625, 1.1577705078125, 0.1215380859375, -1.500345703125, -1.112474609375, 2.0203779296875])
            traj.append_waypoint(waypoint.to_msg())

            # neutral position
            waypoint.set_joint_angles([-0.155232421875, 0.4621865234375, -0.3448271484375, 0.4330361328125, 0.017708984375, -0.946375, 2.040958984375])
            traj.append_waypoint(waypoint.to_msg())

        # set the interaction control options in the current configuration
        interaction_options = InteractionOptions()
        trajectory_options = TrajectoryOptions()
        trajectory_options.interaction_control = True
        trajectory_options.interpolation_type = args.trajType

        interaction_options.set_interaction_control_active(int2bool(args.interaction_active))
        interaction_options.set_K_impedance(args.K_impedance)
        interaction_options.set_max_impedance(int2bool(args.max_impedance))
        interaction_options.set_interaction_control_mode(args.interaction_control_mode)
        interaction_options.set_in_endpoint_frame(int2bool(args.in_endpoint_frame))
        interaction_options.set_force_command(args.force_command)
        interaction_options.set_K_nullspace(args.K_nullspace)
        interaction_options.set_endpoint_name(args.endpoint_name)
        if len(args.interaction_frame) < 7:
            rospy.logerr('The number of elements must be 7!')
        elif len(args.interaction_frame) == 7:
            quat_sum_square = args.interaction_frame[3]*args.interaction_frame[3] + args.interaction_frame[4]*args.interaction_frame[4]
            + args.interaction_frame[5]*args.interaction_frame[5] + args.interaction_frame[6]*args.interaction_frame[6]
            if quat_sum_square  < 1.0 + 1e-7 and quat_sum_square > 1.0 - 1e-7:
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
                rospy.logerr('Invalid input to quaternion! The quaternion must be a unit quaternion!')
        else:
            rospy.logerr('Invalid input to interaction_frame!')

        interaction_options.set_disable_damping_in_force_control(args.disable_damping_in_force_control)
        interaction_options.set_disable_reference_resetting(args.disable_reference_resetting)
        interaction_options.set_rotations_for_constrained_zeroG(args.rotations_for_constrained_zeroG)

        trajectory_options.interaction_params = interaction_options.to_msg()
        traj.set_trajectory_options(trajectory_options)

        gd = GetData()
        gd.start_recording()

        result = traj.send_trajectory(timeout=args.timeout)
        if result is None:
            rospy.logerr('Trajectory FAILED to send!')
            return

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory with interaction options set!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)

        # print the resultant interaction options
        rospy.loginfo('Interaction Options:\n%s', interaction_options.to_msg())

        gd.stop_recording()
        gd.convertandsave('motion1_all_30deg_35N')

    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. %s',
                     'Exiting before trajectory completion.')

if __name__ == '__main__':
    main()
