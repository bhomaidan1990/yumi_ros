#!/usr/bin/python3

import sys
from math import pi as PI
import rospy
import tf
from std_msgs.msg import Float64
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from moveit_msgs.msg import *
from moveit_commander import *
from rospy_message_converter import message_converter
from yumi_hw.srv import *

LEFT  = 1  # :ID of the left arm
RIGHT = 2  # :ID of the right arm
BOTH  = 3  # :ID of both_arms

global group_l  # :The move group for the left arm
global group_r  # :The move group for the right arm
global group_both  # :The move group for using both arms at once
global robot  # :The RobotCommander() from MoveIt!
global scene  # :The PlanningSceneInterface from MoveIt!

# Initializes the package to interface with MoveIt!
def init_Moveit(planner="ESTkConfigDefault", planning_attempts=100, planning_time=50):
    """Initializes the connection to MoveIt!

    Initializes all the objects related to MoveIt! functions. Also adds in the
    table to the MoveIt! planning scene.

    :returns: Nothing
    :rtype: None
    """

    global group_l
    global group_r
    global group_both
    global robot
    global scene
    print("####################################     Start Initialization     ####################################")
    roscpp_initialize(sys.argv)

    robot = RobotCommander()
    scene = PlanningSceneInterface()
    mpr = MotionPlanRequest()
    rospy.sleep(1.0)

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    #===========
    # Left arm |
    #===========    
    # group_l = moveit_commander.MoveGroupCommander("left_arm")
    group_l = MoveGroupCommander("left_arm")
    # Type of planner
    group_l.set_planner_id(planner)
    group_l.set_pose_reference_frame("yumi_body")

    # Setting the workspace
    # group_l.set_workspace(ws=ws_L)

    # Replanning
    group_l.allow_replanning(True)
    group_l.set_goal_tolerance(0.005)
    group_l.set_num_planning_attempts(planning_attempts)
    group_l.set_planning_time(planning_time)
    print('For the Left arm the end effector link is')
    print(group_l.get_end_effector_link())
    print(group_l.get_planning_frame())

    #============
    # Right arm |
    #============
    # group_r = moveit_commander.MoveGroupCommander("right_arm")
    group_r = MoveGroupCommander("right_arm")
    # Type of planner
    group_r.set_planner_id(planner)
    group_r.set_pose_reference_frame("yumi_body")

    # Replanning
    group_r.allow_replanning(True)
    group_r.set_goal_tolerance(0.005)
    group_r.set_num_planning_attempts(planning_attempts)
    group_r.set_planning_time(planning_time)
    print('For the Right arm the end effector link is')
    print(group_r.get_end_effector_link())
    print(group_l.get_planning_frame())

    #============
    # Both arms |
    #============
    group_both = MoveGroupCommander("both_arms")
    # Type of planner
    group_both.set_planner_id(planner)

    # Pose reference frame is the yumi_body
    group_both.set_pose_reference_frame("yumi_body")
    # Replanning
    group_both.allow_replanning(True)
    group_both.set_goal_tolerance(0.005)
    group_both.set_num_planning_attempts(planning_attempts)
    group_both.set_planning_time(planning_time)

    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path', DisplayTrajectory, queue_size=20)
    rospy.sleep(3)
    print("####################################     Finished Initialization     ####################################")

    sys.stdout.write('\nYuMi MoveIt! motion initialized!\n\n\n')

# Set the gripper to an effort value
def gripper_effort(gripper_id, effort):
    """Set gripper effort

    Sends an effort command to the selected gripper. Should be in the range of
    -20.0 (fully open) to 20.0 (fully closed)

    :param gripper_id: The ID of the selected gripper (LEFT or RIGHT)
    :param effort: The effort value for the gripper (-20.0 to 20.0)
    :type gripper_id: int
    :type effort: float
    :returns: Nothing
    :rtype: None
    """
    rospy.loginfo("Setting gripper " + str(gripper_id) + " to " + str(effort))
    rospy.loginfo('Setting gripper effort to ' + str(effort) + ' for arm ' + str(gripper_id))

    if gripper_id == RIGHT:
        pubname = '/yumi/gripper_r_effort_cmd'
        pub = rospy.Publisher(pubname, Float64, queue_size=10, latch=True)
        pub.publish(Float64(effort))
    elif gripper_id == LEFT:
        pubname = '/yumi/gripper_l_effort_cmd'
        pub = rospy.Publisher(pubname, Float64, queue_size=10, latch=True)
        pub.publish(Float64(effort))
    elif gripper_id == BOTH:
        pubname = '/yumi/gripper_l_effort_cmd'
        pub = rospy.Publisher(pubname, Float64, queue_size=10, latch=True)
        pub.publish(Float64(effort))
        pubname = '/yumi/gripper_r_effort_cmd'
        pub = rospy.Publisher(pubname, Float64, queue_size=10, latch=True)
        pub.publish(Float64(effort))
    else:
        print("Worng arm option, please choose LEFT, RIGHT. or BOTH")
        rospy.logwarn("Wrong arm option!")

    rospy.sleep(1.0)

def grasp(arm, grip_effort):
    """Grasp

    Sets the gripper effort to Open/Close the gripper

    :param arm: The selected arm (LEFT, RIGHT or BOTH)
    :param grip_effort: gripping effort.
    :type arm: int
    :type grip_effort: int
    :returns: Nothing
    :rtype: None
    """
    if (grip_effort <= 20 and grip_effort >= -20):
        gripper_effort(arm, grip_effort)
    else:
        print("The gripper effort values should be in the range [-20, 20]")

# Make a plan and move within the joint space
def go_to_joints(positions, arm):
    """Set joint values

    Moves the selected arm to make the joint positions match the given values

    :param positions: The desired joint values [j1-j7] (or [j1l-j7l,j1r-j7r] for both arms at once)
    :param arm: The selected arm (LEFT, RIGHT or BOTH)
    :type positions: float[]
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    global group_l
    global group_r
    global group_both

    cur_arm = group_l
    if arm == RIGHT:
        cur_arm = group_r
    elif arm == BOTH:
        cur_arm = group_both

    if arm == BOTH:
        cur_arm.set_joint_value_target(positions[0] + positions[1])
    else:
        cur_arm.set_joint_value_target(positions)
    cur_arm.plan()
    cur_arm.go(wait=True)
    rospy.sleep(3)

if __name__ == '__main__':
    try:
        rospy.init_node('yumi_motion_api', anonymous=True)
        rospy.loginfo('Yumi_otion_API node Initialization')

        rate = rospy.Rate(10.0)
        args = rospy.myargv(sys.argv)

        print(len(sys.argv), len(args))

        if len(args) < 3:
            print("usage: yumi_motion_api.py joints positions arm  [optional grip effort]")
            rospy.logwarn("yumi_motion_api node missing parameters!")
        elif len(args) > 4:
            print("usage: yumi_motion_api.py joints positions arm  [optional grip effort]")
            rospy.logwarn("yumi_motion_api node Extra parameters!")  
        else:
            init_Moveit()
            joints_str = args[1].split(",")
            joints_str[0]=joints_str[0][1:]
            joints_str[-1]=joints_str[-1][:-1]
            joints = [float(i) for i in joints_str]
            armID  = int(args[2])
            go_to_joints(joints, armID)
            if(len(args)==4):
                effort = int(args[3])
                grasp(armID, effort)
            
            sys.exit()
            
    except rospy.ROSInterruptException:
        rospy.logerr("yumi_motion_api node has died!")