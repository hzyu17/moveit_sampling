# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import JointState

    
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def printInfo(move_group):
    planning_frame = move_group.get_planning_frame()
    print("=== planning frame: %s" % planning_frame)

    planning_pipeline = move_group.get_planning_pipeline_id()
    print("=== planning pipeline: %s" % planning_pipeline)
 
    eef_link = move_group.get_end_effector_link()
    print("=== end effector link: %s" % eef_link)

    print("")

def printRobotInfo(robot):
    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

# instantiate the robot
robot = moveit_commander.RobotCommander()

# instantiate the scene
scene = moveit_commander.PlanningSceneInterface()

# move_group
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.allow_replanning(True)
move_group.set_num_planning_attempts(10)

# display trajectory publisher
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

# We get the joint values from the group and change some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -tau / 8
joint_goal[2] = 0
joint_goal[3] = -tau / 5
joint_goal[4] = 0
joint_goal[5] = tau / 6  # 1/6 of a turn
joint_goal[6] = 0

# set start state to current state
move_group.set_start_state_to_current_state()

# Set the planning pipeline
#move_group.set_planning_pipeline_id("chomp")
#move_group.set_planning_time(5)
#move_group.set_planner_id("RRTConnect")

# print move_group information
printInfo(move_group)

# Plan
joint_goal_state = JointState()
joint_goal_state.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
joint_goal_state.position = joint_goal

move_group.construct_motion_plan_request()
plan = move_group.plan(joint_goal_state)

print("============ Plan:")
print(plan)

if (plan[0]):
    print("============ plan succeeded:")

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
#plan = move_group.go(joint_goal, wait=True)
plan = move_group.go(wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()
move_group.clear_pose_targets()

