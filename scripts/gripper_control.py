#!/usr/bin/env python
import rospy
import actionlib
from franka_gripper.msg import HomingAction, HomingGoal
from franka_gripper.msg import MoveAction, MoveGoal
from franka_gripper.msg import GraspAction, GraspGoal

def do_homing():
    rospy.loginfo("Sending homing command...")
    client = actionlib.SimpleActionClient('/franka_gripper/homing', HomingAction)
    client.wait_for_server()
    goal = HomingGoal()
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Homing done.")

def open_gripper(width=0.08, speed=0.1):
    rospy.loginfo("Opening gripper...")
    client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    client.wait_for_server()
    goal = MoveGoal(width=width, speed=speed)
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Gripper opened.")

def grasp_object(width=0.04, speed=0.7, force=100.0):
    rospy.loginfo("Grasping object...")
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    client.wait_for_server()
    goal = GraspGoal()
    goal.width = width
    goal.speed = speed
    goal.force = force
    goal.epsilon.inner = 0.08
    goal.epsilon.outer = 0.08
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo("Grasp result: {}".format(result))
    rospy.loginfo("Grasp success: {}".format(result.success))

if __name__ == "__main__":
    rospy.init_node("franka_gripper_control_node")

    do_homing()
    rospy.sleep(1.0)

    open_gripper()
    rospy.sleep(1.0)

    grasp_object()
    rospy.sleep(1.0)

    #open_gripper()

