#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
from math import pi
from geometry_msgs.msg import Pose

def get_current_pose(group_arm): #現在のtcpの位置姿勢を取得
  current_pose = group_arm.get_current_pose("tcp").pose
  return current_pose

def home(group_arm):
  MS_cubic_home = [0, -2, 2, -1, 0, pi-2] #['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'ee_fixed_joint']
  group_arm.set_joint_value_target(MS_cubic_home)
  group_arm.go()

def home_s(group_arm): #ホームポジションに真っ直ぐ戻る関数
  current_pose = get_current_pose(group_arm)
  group_arm.set_pose_target(home_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, home_pose], 0.01, 0.0) #直線パスの生成
  group_arm.execute(plan)
  

def move_X(group_arm, distance): #global座標
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x + distance
  target_pose.position.y = current_pose.position.y
  target_pose.position.z = current_pose.position.z
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0) #直線パスの生成
  group_arm.execute(plan)


def move_Y(group_arm, distance): #global座標
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x
  target_pose.position.y = current_pose.position.y + distance
  target_pose.position.z = current_pose.position.z
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  group_arm.execute(plan)


def move_Z(group_arm, distance): #global座標
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x
  target_pose.position.y = current_pose.position.y
  target_pose.position.z = current_pose.position.z + distance
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  group_arm.execute(plan)



def mover(group_arm):
  home(group_arm)
  move_X(group_arm,0.1)
  rospy.sleep(1.0)
  group_arm.set_max_velocity_scaling_factor(0.001)
  move_X(group_arm,0.1)


if __name__ == '__main__':
  try:
    rospy.init_node('move_group_interface_tutorial2')
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander("ur5_arm") 

    group_arm.set_max_velocity_scaling_factor(1.0)

    mover(group_arm)

    rospy.spin()

  except rospy.ROSInterruptException:
    pass

