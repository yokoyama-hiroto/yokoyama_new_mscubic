#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import time
import tf
import geometry_msgs.msg
import math
import numpy as np
from math import pi
import threading
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, translation_matrix, concatenate_matrices, translation_from_matrix, quaternion_from_matrix
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Vector3

sleep_time = 0.01 #sleeping time
#distance = 0.1 #移動距離
#angle = pi/9   #移動角度
home_pose = Pose()
threshold = 50 #Power
offset = [0.07, 0.03, 0]

def get_current_pose(group_arm): #現在のtcpの位置姿勢を取得
  current_pose = group_arm.get_current_pose("tool0").pose
  return current_pose

def force_sensor_callback(msg): #センサー値を取得するコールバック関数
    global force_sensor_value_x
    global force_sensor_value_y
    global force_sensor_value_z
    force_sensor_value_x = msg.wrench.force.x
    force_sensor_value_y = msg.wrench.force.y
    force_sensor_value_z = msg.wrench.force.z
    print("\r" + f'{force_sensor_value_x:6.1f}' +" " +  f'{force_sensor_value_y:6.1f}' + " " +  f'{force_sensor_value_z:6.1f}', end="")

def get_sensor_average(): #センサー値の時間平均を算出
  fx = 0
  fy = 0
  fz = 0
  for i in range(10):
    fx += force_sensor_value_x
    fy += force_sensor_value_y
    fz += force_sensor_value_z
    rospy.sleep(sleep_time/10)
  f = [fx/10, fy/10, fz/10]
  return f

def get_force_size(f): #力の大きさを計算
   F_size = math.sqrt(f[0]**2 + f[1]**2 + f[2]**2)
   return F_size

def force_ratio(f): #センサー値の成分比率を算出
  fx = f[0]/math.sqrt(f[0]**2+f[1]**2+f[2]**2)
  fy = f[1]/math.sqrt(f[0]**2+f[1]**2+f[2]**2)
  fz = f[2]/math.sqrt(f[0]**2+f[1]**2+f[2]**2)
  f_ratio = [fx,fy,fz]
  return f_ratio

def execute_movement(group_arm, plan):
  """
  for i in range(len(plan.joint_trajectory.points)):
    group_arm.set_joint_value_target(plan.joint_trajectory.points[i].positions)
    group_arm.go()
  """
  group_arm.execute(plan)

def scaling_plan(plan, scale):
  del plan.joint_trajectory.points[0]
  for i in range(len(plan.joint_trajectory.points)):
    #print(plan.joint_trajectory.points[i])
    velocity = plan.joint_trajectory.points[i].velocities
    scaledvelocity = [n*scale for n in velocity]
    plan.joint_trajectory.points[i].velocities = scaledvelocity
    acceleration = plan.joint_trajectory.points[i].accelerations
    scaledacceleration = [n*scale for n in acceleration]
    plan.joint_trajectory.points[i].accelerations = scaledacceleration
    plan.joint_trajectory.points[i].time_from_start = plan.joint_trajectory.points[i].time_from_start / scale
  
  return plan

def euler_to_quaternion(euler):
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]) 

def quaternion_to_euler(quaternion):
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def home(group_arm):
  MS_cubic_home = [0, -2, 2, -1.2, 0, 1.2-pi] #['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'ee_fixed_joint']
  group_arm.set_joint_value_target(MS_cubic_home)
  group_arm.go()
  
"""
def home_s(group_arm): #ホームポジションに真っ直ぐ戻る関数
  current_pose = get_current_pose(group_arm)
  group_arm.set_pose_target(home_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, home_pose], 0.01, 0.0) #直線パスの生成
  group_arm.execute(plan)
"""

def force_threshold(initial_force):
  force = 0
  while force < threshold:
     force = get_force_size([x - y for x, y in zip(get_sensor_average(), initial_force)])
     #print([x - y for x, y in zip(get_sensor_average(), initial_force)])
  else:
      print("Finish touching")
      group_arm.stop()

def move_X(group_arm, distance):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x + distance
  target_pose.position.y = current_pose.position.y
  target_pose.position.z = current_pose.position.z
  target_pose.orientation = current_pose.orientation

  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.001, 0.0) #直線パスの生成
  scaling_plan(plan, 1.0)
  execute_movement(group_arm, plan)


def move_Y(group_arm, distance):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x
  target_pose.position.y = current_pose.position.y + distance
  target_pose.position.z = current_pose.position.z
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  execute_movement(group_arm, plan)


def move_Z(group_arm, distance):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x
  target_pose.position.y = current_pose.position.y
  target_pose.position.z = current_pose.position.z + distance
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  execute_movement(group_arm, plan)

def rotate_around_offset_center(group_arm, roll, pitch, yaw):
    # オフセット点を現在の位置に加算
  pose = get_current_pose(group_arm)
  center_point = np.array([pose.position.x, pose.position.y, pose.position.z]) + np.array(offset)
    
    # 現在の位置を中心点に移動
  translation_to_center = translation_matrix(-center_point)
  translation_back = translation_matrix(center_point)
    
    # Roll, Pitch, Yaw 回転のクォータニオンを生成
  q_rot = quaternion_from_euler(roll, pitch, yaw)
  rotation = quaternion_matrix(q_rot)
    
    # 現在の姿勢を行列に変換
  current_translation = translation_matrix((pose.position.x, pose.position.y, pose.position.z))
  current_orientation = quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    
    # 中心点での回転を行う
  new_matrix = concatenate_matrices(translation_to_center, rotation, translation_back, current_orientation)
    
    # 行列を位置とクォータニオンに戻す
  new_translation = translation_from_matrix(new_matrix)
  new_quaternion = quaternion_from_matrix(new_matrix)
    
    # 新しいポーズを返す
  new_pose = Pose()
  new_pose.position.x, new_pose.position.y, new_pose.position.z = new_translation[:3]
  new_pose.orientation.x, new_pose.orientation.y, new_pose.orientation.z, new_pose.orientation.w = new_quaternion
  (plan, fraction) = group_arm.compute_cartesian_path([pose, new_pose], 0.01, 0.0, avoid_collisions=False)
  print(fraction)
  scaling_plan(plan, 0.2)
  execute_movement(group_arm, plan)


def move_roll(group_arm, angle):
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
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.001, 0.0)
  scaling_plan(plan, 0.2)
  execute_movement(group_arm, plan)
  """
  plan = rotate_around_offset_center(group_arm, angle, 0, 0)
  execute_movement(group_arm, plan)
  """

def move_pitch(group_arm, angle):
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
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.001, 0.0)
  scaling_plan(plan, 0.2)
  execute_movement(group_arm, plan)
  """
  plan = rotate_around_offset_center(group_arm, 0, angle, 0)
  execute_movement(group_arm, plan)
  """

def move_yaw(group_arm, angle):
  
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
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.001, 0.0)
  scaling_plan(plan, 0.2)
  execute_movement(group_arm, plan)
  """
  plan = rotate_around_offset_center(group_arm, 0, 0, angle)
  execute_movement(group_arm, plan)
  """
  
def move_x(group_arm, distance): #手先座標 -xがドリル軸進行方向
    current_pose = get_current_pose(group_arm)
    # 手先座標系での目標位置と姿勢を定義
    goal_position_hand = [distance, 0.0, 0.0]  # x方向にdistanceだけ移動
    goal_orientation_hand = [0.0, 0.0, 0.0, 1.0]  # 手先座標系の姿勢は変わらないと仮定

    # 現在のエンドエフェクタの位置と姿勢を抽出
    current_position = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
    current_orientation = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]

    # 現在のエンドエフェクタの変換行列 (グローバル座標系)
    current_transformation = concatenate_matrices(translation_matrix(current_position), quaternion_matrix(current_orientation))
    
    # 手先座標系での目標位置と姿勢の変換行列
    goal_transformation_hand = concatenate_matrices(translation_matrix(goal_position_hand), quaternion_matrix(goal_orientation_hand))
    
    # グローバル座標系への変換
    goal_transformation_global = concatenate_matrices(current_transformation, goal_transformation_hand)
    
    # 目標位置と姿勢をグローバル座標系に変換
    goal_position_global = goal_transformation_global[:3, 3]
    goal_orientation_global = tf.transformations.quaternion_from_matrix(goal_transformation_global)
    
    # geometry_msgs.msg.Pose形式で返す
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = goal_position_global[0]
    target_pose.position.y = goal_position_global[1]
    target_pose.position.z = goal_position_global[2]
    target_pose.orientation.x = goal_orientation_global[0]
    target_pose.orientation.y = goal_orientation_global[1]
    target_pose.orientation.z = goal_orientation_global[2]
    target_pose.orientation.w = goal_orientation_global[3]

    group_arm.set_pose_target(target_pose)
    (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.0001, 0.0)
    scaling_plan(plan, 0.2)
    execute_movement(group_arm, plan)

def move_y(group_arm, distance): #手先座標
    current_pose = get_current_pose(group_arm)
    # 手先座標系での目標位置と姿勢を定義
    goal_position_hand = [0.0, distance, 0.0]  # x方向にdistanceだけ移動
    goal_orientation_hand = [0.0, 0.0, 0.0, 1.0]  # 手先座標系の姿勢は変わらないと仮定

    # 現在のエンドエフェクタの位置と姿勢を抽出
    current_position = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
    current_orientation = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]

    # 現在のエンドエフェクタの変換行列 (グローバル座標系)
    current_transformation = concatenate_matrices(translation_matrix(current_position), quaternion_matrix(current_orientation))
    
    # 手先座標系での目標位置と姿勢の変換行列
    goal_transformation_hand = concatenate_matrices(translation_matrix(goal_position_hand), quaternion_matrix(goal_orientation_hand))
    
    # グローバル座標系への変換
    goal_transformation_global = concatenate_matrices(current_transformation, goal_transformation_hand)
    
    # 目標位置と姿勢をグローバル座標系に変換
    goal_position_global = goal_transformation_global[:3, 3]
    goal_orientation_global = tf.transformations.quaternion_from_matrix(goal_transformation_global)
    
    # geometry_msgs.msg.Pose形式で返す
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = goal_position_global[0]
    target_pose.position.y = goal_position_global[1]
    target_pose.position.z = goal_position_global[2]
    target_pose.orientation.x = goal_orientation_global[0]
    target_pose.orientation.y = goal_orientation_global[1]
    target_pose.orientation.z = goal_orientation_global[2]
    target_pose.orientation.w = goal_orientation_global[3]

    group_arm.set_pose_target(target_pose)
    (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
    #scaling_plan(plan, 0.1)
    execute_movement(group_arm, plan)

def move_z(group_arm, distance): #手先座標
    current_pose = get_current_pose(group_arm)
    # 手先座標系での目標位置と姿勢を定義
    goal_position_hand = [0.0, 0.0, distance]  # x方向にdistanceだけ移動
    goal_orientation_hand = [0.0, 0.0, 0.0, 1.0]  # 手先座標系の姿勢は変わらないと仮定

    # 現在のエンドエフェクタの位置と姿勢を抽出
    current_position = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
    current_orientation = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]

    # 現在のエンドエフェクタの変換行列 (グローバル座標系)
    current_transformation = concatenate_matrices(translation_matrix(current_position), quaternion_matrix(current_orientation))
    
    # 手先座標系での目標位置と姿勢の変換行列
    goal_transformation_hand = concatenate_matrices(translation_matrix(goal_position_hand), quaternion_matrix(goal_orientation_hand))
    
    # グローバル座標系への変換
    goal_transformation_global = concatenate_matrices(current_transformation, goal_transformation_hand)
    
    # 目標位置と姿勢をグローバル座標系に変換
    goal_position_global = goal_transformation_global[:3, 3]
    goal_orientation_global = tf.transformations.quaternion_from_matrix(goal_transformation_global)
    
    # geometry_msgs.msg.Pose形式で返す
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = goal_position_global[0]
    target_pose.position.y = goal_position_global[1]
    target_pose.position.z = goal_position_global[2]
    target_pose.orientation.x = goal_orientation_global[0]
    target_pose.orientation.y = goal_orientation_global[1]
    target_pose.orientation.z = goal_orientation_global[2]
    target_pose.orientation.w = goal_orientation_global[3]

    group_arm.set_pose_target(target_pose)
    (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
    #scaling_plan(plan, 0.1)
    execute_movement(group_arm, plan)

def next_angle_yaw(θ1,f0,f1):
   θ2 = θ1*f1[1]/(f1[1]-f0[1])
   return θ2

def next_angle_pitch(θ1,f0,f1):
   θ2 = θ1*f1[2]/(f1[2]-f0[2])
   return θ2

def angle_correction_yaw(group_arm):
   f0 = force_ratio([x - y for x, y in zip(get_sensor_average(), initial_force)])
   θ1 = math.degrees(math.atan(f0[1]/f0[0])) #arctanから1回目の回転角を設定
   print("θ1=",θ1)
   move_yaw(group_arm,θ1)
   rospy.sleep(sleep_time)
   f1 = force_ratio([x - y for x, y in zip(get_sensor_average(), initial_force)])
   θ2 = next_angle_yaw(θ1,f0,f1)
   print("θ2=",θ2)
   move_yaw(group_arm,θ2)
   rospy.sleep(sleep_time)
   f2 = force_ratio([x - y for x, y in zip(get_sensor_average(), initial_force)])
   θ3 = next_angle_yaw(θ2,f1,f2)
   print("θ3=",θ3)
   move_yaw(group_arm,θ3)
   rospy.sleep(sleep_time)
   f3 = force_ratio([x - y for x, y in zip(get_sensor_average(), initial_force)])
   θ4 = next_angle_yaw(θ3,f2,f3)
   print("θ4=",θ4)
   move_yaw(group_arm,θ4)
   print("Finish correction yaw!")

def angle_correction_pitch(group_arm):
   f0 = force_ratio([x - y for x, y in zip(get_sensor_average(), initial_force)])
   θ1 = math.degrees(math.atan(f0[2]/f0[0])) #arctanから1回目の回転角を設定
   print("θ1=",θ1)
   move_pitch(group_arm,θ1)
   rospy.sleep(sleep_time)
   f1 = force_ratio([x - y for x, y in zip(get_sensor_average(), initial_force)])
   θ2 = next_angle_pitch(θ1,f0,f1)
   print("θ2=",θ2)
   move_pitch(group_arm,θ2)
   rospy.sleep(sleep_time)
   f2 = force_ratio([x - y for x, y in zip(get_sensor_average(), initial_force)])
   θ3 = next_angle_pitch(θ2,f1,f2)
   print("θ3=",θ3)
   move_pitch(group_arm,θ3)
   rospy.sleep(sleep_time)
   f3 = force_ratio([x - y for x, y in zip(get_sensor_average(), initial_force)])
   θ4 = next_angle_pitch(θ3,f2,f3)
   print("θ4=",θ4)
   move_pitch(group_arm,θ4)
   print("Finish correction pitch!")



def mover(group_arm):
  home(group_arm)
  rospy.sleep(0.1)
  global initial_force
  initial_force = get_sensor_average()
  rospy.sleep(1)
  print("Finish setting initial")
  #force_thread = threading.Thread(target=force_threshold(initial_force))
  #move_thread = threading.Thread(target=move_x(group_arm, 0.5))
  #force_thread.start()
  #print("aaa")
  
  #move_thread.start()
  """
  force = 0
  i = 0
  while force < threshold:
     move_X(group_arm, 0.01)
     force = get_force_size([x - y for x, y in zip(get_sensor_average(), initial_force)])
     #print([x - y for x, y in zip(get_sensor_average(), initial_force)])
     #print(force)
     print(i)
     i=i+1
  else:
      print("Finish touching")
      rospy.sleep(1)
      print(group_arm.get_current_joint_values())
      angle_correction_yaw(group_arm)
      angle_correction_pitch(group_arm)
  """
  move_X(group_arm, 0.5)
  print("finish touching")
  rospy.sleep(3)
  angle_correction_yaw(group_arm)
  angle_correction_pitch(group_arm)


def move(group_arm):
  home(group_arm)
  move_X(group_arm, 0.2)
  #rotate_around_offset_center(group_arm, 0, pi/16, 0)


if __name__ == '__main__':
  try:
    rospy.init_node('move_group_interface_tutorial2')
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander("ur5_arm") 

    group_arm.set_max_velocity_scaling_factor(1.0)
    group_arm.set_max_acceleration_scaling_factor(1.0)

    # 力センサのトピックを購読し、コールバック関数を設定
    rospy.Subscriber('ft_sensor/raw', WrenchStamped, force_sensor_callback)


    # ロボットの制御を行うスレッドを開始
    control_thread = threading.Thread(target=move(group_arm))
    control_thread.start()

    rospy.spin()

  except rospy.ROSInterruptException:
    pass

