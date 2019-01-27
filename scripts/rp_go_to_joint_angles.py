import rospy
import argparse
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_interface import Limb

def main():
    """
    Move the robot arm to the specified configuration.
    Call using:
    $ rosrun intera_examples go_to_joint_angles.py  [arguments: see below]

    -q 0.0 0.0 0.0 0.0 0.0 0.0 0.0
    --> Go to joint pose: 0.0 0.0 0.0 0.0 0.0 0.0 0.0 using default settings

    -q 0.1 -0.2 0.15 -0.05 -0.08 0.14 -0.04 -s 0.1
    --> Go to pose [...] with a speed ratio of 0.1

    -q -0.2 0.1 0.1 0.2 -0.3 0.2 0.4 -s 0.9 -a 0.1
    --> Go to pose [...] with a speed ratio of 0.9 and an accel ratio of 0.1
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        "-q", "--joint_angles", type=float,
        nargs='+', default=[0.158984375, 0.665759765625, -1.53172265625, 1.0492724609375, 0.8098212890625, -1.0504248046875, 2.89727734375],
        help="A list of joint angles, one for each of the 7 joints, J0...J6")
    parser.add_argument(
        "-s",  "--speed_ratio", type=float, default=0.1,
        help="A value between 0.001 (slow) and 1.0 (maximum joint velocity)")
    parser.add_argument(
        "-a",  "--accel_ratio", type=float, default=0.1,
        help="A value between 0.001 (slow) and 1.0 (maximum joint accel)")
    parser.add_argument(
        "--timeout", type=float, default=None,
        help="Max time in seconds to complete motion goal before returning. None is interpreted as an infinite timeout.")
    args = parser.parse_args(rospy.myargv()[1:])

    try:
        rospy.init_node('go_to_joint_angles_py')
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

        # slight right (20 degree)
        # waypoint.set_joint_angles([-0.155232421875, 0.4621865234375, -0.3448271484375, 0.4330361328125, 0.017708984375, -0.946375, 3])
        # traj.append_waypoint(waypoint.to_msg())
        #
        # # neutral position
        # waypoint.set_joint_angles([-0.155232421875, 0.4621865234375, -0.3448271484375, 0.4330361328125, 0.017708984375, -0.946375, 2.040958984375])
        # traj.append_waypoint(waypoint.to_msg())
        #
        # # slight left (20 degree)
        # waypoint.set_joint_angles([0.178685546875, -0.2291533203125, -0.7179814453125, 1.633763671875, 2.1484375e-05, -1.3499716796875, 2.0902988281250003])
        # traj.append_waypoint(waypoint.to_msg())
        #
        # # neutral position
        # waypoint.set_joint_angles([0.178685546875, -0.2291533203125, -0.7179814453125, 1.633763671875, 2.1484375e-05, -1.3499716796875, 2.439298828125])
        # traj.append_waypoint(waypoint.to_msg())
        #
        # # slight front (20 degree)
        # waypoint.set_joint_angles([0.1974599609375, -0.27071484375, -0.7559970703125, 1.5779091796875, -0.14858203125, -1.1271669921875, 2.5262158203125])
        # traj.append_waypoint(waypoint.to_msg())
        #
        # # neutral position
        # waypoint.set_joint_angles([0.178685546875, -0.2291533203125, -0.7179814453125, 1.633763671875, 2.1484375e-05, -1.3499716796875, 2.439298828125])
        # traj.append_waypoint(waypoint.to_msg())
        #
        # # slight back (20 degree)
        # waypoint.set_joint_angles([0.1611396484375, -0.174541015625, -0.6814091796875, 1.6546083984375, 0.1373291015625, -1.5653515625, 2.39933984375])
        # traj.append_waypoint(waypoint.to_msg())
        #
        # # neutral position
        # waypoint.set_joint_angles([0.178685546875, -0.2291533203125, -0.7179814453125, 1.633763671875, 2.1484375e-05, -1.3499716796875, 2.439298828125])
        # traj.append_waypoint(waypoint.to_msg())


        result = traj.send_trajectory(timeout=args.timeout)
        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')


if __name__ == '__main__':
    main()
