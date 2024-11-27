#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import time
import tf
import geometry_msgs.msg
from math import pi
import math
import threading
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

sleep_time = 1 #sleeping time
distance = 0.1 #移動距離
angle = pi/9   #移動角度
home_pose = Pose()

def get_current_pose(group_arm):
  current_pose = group_arm.get_current_pose("tool0").pose
  return current_pose



def euler_to_quaternion(euler):
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]) 

def quaternion_to_euler(quaternion):
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def home(group_arm):
  MS_cubic_home = [0, -1.5, 1.5, 0, pi/2, -pi/2] #['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'ee_fixed_joint']
  group_arm.set_joint_value_target(MS_cubic_home)
  group_arm.go()
  

def home_s(group_arm): #ホームポジションに真っ直ぐ戻る関数
  current_pose = get_current_pose(group_arm)
  group_arm.set_pose_target(home_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, home_pose], 0.01, 0.0) #直線パスの生成
  group_arm.execute(plan)
  

def move_X_up(group_arm):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x + distance
  target_pose.position.y = current_pose.position.y
  target_pose.position.z = current_pose.position.z
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0) #直線パスの生成
  print(plan)
  group_arm.execute(plan)

def move_X_down(group_arm):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x - distance
  target_pose.position.y = current_pose.position.y
  target_pose.position.z = current_pose.position.z
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  group_arm.execute(plan)

def move_Y_up(group_arm):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x
  target_pose.position.y = current_pose.position.y + distance
  target_pose.position.z = current_pose.position.z
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  group_arm.execute(plan)

def move_Y_down(group_arm):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x
  target_pose.position.y = current_pose.position.y - distance
  target_pose.position.z = current_pose.position.z
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  group_arm.execute(plan)

def move_Z_up(group_arm):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x
  target_pose.position.y = current_pose.position.y
  target_pose.position.z = current_pose.position.z + distance
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  group_arm.execute(plan)

def move_Z_down(group_arm):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x
  target_pose.position.y = current_pose.position.y
  target_pose.position.z = current_pose.position.z - distance
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  group_arm.execute(plan)

def move_roll_up(group_arm):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position = current_pose.position
  euler = quaternion_to_euler(current_pose.orientation)
  euler.x = euler.x + angle
  quaternion = euler_to_quaternion(euler)
  target_pose.orientation.x = quaternion.x
  target_pose.orientation.y = quaternion.y
  target_pose.orientation.z = quaternion.z
  target_pose.orientation.w = quaternion.w

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  group_arm.execute(plan)

def move_roll_down(group_arm):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position = current_pose.position
  euler = quaternion_to_euler(current_pose.orientation)
  euler.x = euler.x - angle
  quaternion = euler_to_quaternion(euler)
  target_pose.orientation.x = quaternion.x
  target_pose.orientation.y = quaternion.y
  target_pose.orientation.z = quaternion.z
  target_pose.orientation.w = quaternion.w

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  group_arm.execute(plan)

def move_pitch_up(group_arm):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position = current_pose.position
  euler = quaternion_to_euler(current_pose.orientation)
  euler.y = euler.y + angle
  quaternion = euler_to_quaternion(euler)
  target_pose.orientation.x = quaternion.x
  target_pose.orientation.y = quaternion.y
  target_pose.orientation.z = quaternion.z
  target_pose.orientation.w = quaternion.w

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  group_arm.execute(plan)

def move_pitch_down(group_arm):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position = current_pose.position
  euler = quaternion_to_euler(current_pose.orientation)
  euler.y = euler.y - angle
  quaternion = euler_to_quaternion(euler)
  target_pose.orientation.x = quaternion.x
  target_pose.orientation.y = quaternion.y
  target_pose.orientation.z = quaternion.z
  target_pose.orientation.w = quaternion.w

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  group_arm.execute(plan)

def move_yaw_up(group_arm):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position = current_pose.position
  euler = quaternion_to_euler(current_pose.orientation)
  euler.z = euler.z + angle
  quaternion = euler_to_quaternion(euler)
  target_pose.orientation.x = quaternion.x
  target_pose.orientation.y = quaternion.y
  target_pose.orientation.z = quaternion.z
  target_pose.orientation.w = quaternion.w

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  group_arm.execute(plan)

def move_yaw_down(group_arm):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position = current_pose.position
  euler = quaternion_to_euler(current_pose.orientation)
  euler.z = euler.z - angle
  quaternion = euler_to_quaternion(euler)
  target_pose.orientation.x = quaternion.x
  target_pose.orientation.y = quaternion.y
  target_pose.orientation.z = quaternion.z
  target_pose.orientation.w = quaternion.w

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  group_arm.execute(plan)

def move_global_system(group_arm, distance): #distanceはglobal座標系
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x + distance[0]
  target_pose.position.y = current_pose.position.y + distance[1]
  target_pose.position.z = current_pose.position.z + distance[2]
  target_pose.orientation = current_pose.orientation

  #group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0) #直線パスの生成
  print(plan)
  group_arm.execute(plan)
  print(fraction)
  return fraction #直線パスの成功率

def move(group_arm, distance): #distanceはglobal座標系
  current_pose = get_current_pose(group_arm) #current_poseが正しいか
  print(current_pose)
  if flag_debug_mode:
    """
    current_pose = Pose()
    current_pose.position.x = 0.5
    current_pose.position.y = 0.5
    current_pose.position.z = 0.5
    """
    current_pose.orientation.x = math.sqrt(2)/2
    current_pose.orientation.y = 0
    current_pose.orientation.z = 0
    current_pose.orientation.w = math.sqrt(2)/2
  




  #print(current_pose)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x + distance[0]
  target_pose.position.y = current_pose.position.y + distance[1]
  target_pose.position.z = current_pose.position.z + distance[2]
  target_pose.orientation = current_pose.orientation

  middle_pose = Pose()
  middle_pose.position.x = current_pose.position.x + distance[0]
  middle_pose.position.y = current_pose.position.y + distance[1]
  middle_pose.position.z = current_pose.position.z + distance[2]/2
  middle_pose.orientation = current_pose.orientation
  #print(target_pose)
  #print(group_arm)

  #group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, middle_pose, target_pose], 0.01, 0.0) #直線パスの生成
  print(plan)
  group_arm.execute(plan)
  print(fraction)
  return fraction #直線パスの成功率


def mover(group_arm):
  home(group_arm)
  #move_global_system(group_arm, [1.0, 0.0, 0.0])
  #move_X_up(group_arm)
  rospy.sleep(1)
  # move(group_arm, [0,0,0.1])
  #move_roll_up(group_arm)
  """
  rospy.sleep(sleep_time)
  home_pose = get_current_pose(group_arm)
  move_roll_up(group_arm)
  print("roll+")
  rospy.sleep(sleep_time)
  home(group_arm)
  rospy.sleep(sleep_time)
  move_roll_down(group_arm)
  print("roll-")
  rospy.sleep(sleep_time)
  home(group_arm)
  rospy.sleep(sleep_time)
  move_pitch_up(group_arm)
  print("pitch+")
  rospy.sleep(sleep_time)
  home(group_arm)
  rospy.sleep(sleep_time)
  move_pitch_down(group_arm)
  print("pitch-")
  rospy.sleep(sleep_time)
  home(group_arm)
  rospy.sleep(sleep_time)
  move_yaw_up(group_arm)
  print("yaw+")
  rospy.sleep(sleep_time)
  home(group_arm)
  rospy.sleep(sleep_time)
  move_yaw_down(group_arm)
  print("yaw-")
  rospy.sleep(sleep_time)
  home(group_arm)
  rospy.sleep(sleep_time)
  """


if __name__ == '__main__':
  try:
    yokoyama = input("1:simulation or 2:real robot")
    if yokoyama == "1":
      ur5_arm = "ur5_arm"
      rostopic_force = "ft_sensor/raw"
    elif yokoyama == "2":
      ur5_arm ="manipulator"
      rostopic_force = "wrench"
    else:
      print("Input 1 or 2")
      sys.exit()

    global flag_debug_mode
    flag_debug_mode = False


    rospy.init_node('move_group_interface_tutorial2')
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander(ur5_arm) 

    group_arm.set_max_velocity_scaling_factor(1.0)
    control_thread = threading.Thread(target=mover(group_arm))
    control_thread.start()

    rospy.spin()

  except rospy.ROSInterruptException:
    pass

