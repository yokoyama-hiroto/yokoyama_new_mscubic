#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import tf
import geometry_msgs.msg
import math
from math import pi
import threading
from tf.transformations import quaternion_matrix, translation_matrix, concatenate_matrices
from geometry_msgs.msg import Pose, WrenchStamped, Vector3, Quaternion


k = 0.002/(0.002 + 1) #ローパスフィルタの係数
f_threshold = 5 #接触力
global group_arm 
global f_current
global f_previous
global f_filtered
global raw_force_value
global filtered_force_value
f_previous = [0, 0, 0]
f_current = [0, 0, 0]
f_filtered = [0, 0, 0]
raw_force_value = [0, 0, 0]
filtered_force_value = [0, 0, 0]

def force_sensor_callback(msg): #センサ値を取得するコールバック関数
  global raw_force_value
  global filtered_force_value
  raw_force_value = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
  filtered_force_value = low_pass_filter(raw_force_value)
  #print("\r" + f'{raw_force_value[0]:6.1f}' +" " +  f'{raw_force_value[1]:6.1f}' + " " +  f'{raw_force_value[2]:6.1f}', end="")
  #print("\r" + f'{filtered_force_value[0]:6.1f}' +" " +  f'{filtered_force_value[1]:6.1f}' + " " +  f'{filtered_force_value[2]:6.1f}', end="")

def low_pass_filter(raw_force_value): #センサ値をローパスフィルタに通す
  global f_current
  global f_previous
  global f_filtered
  f_previous = f_current
  f_current = raw_force_value
  for i in range(3):
     f_filtered[i] = f_previous[i] + (f_current[i] - f_previous[i]) * k
  return f_filtered

def calculate_force_absolute(f): #絶対値を算出
  f_absolute = math.sqrt(f[0]^2 + f[1]^2 + f[2]^2)
  return f_absolute

def calculate_force_ratio(f): #センサー値の成分比率を算出
  f_absolute = calculate_force_absolute(f)
  f_ratio = [n/f_absolute for n in f]
  return f_ratio

def get_current_pose(): #現在のtcpの位置姿勢を取得
  current_pose = group_arm.get_current_pose("tcp").pose #実機では名が違う
  return current_pose

def euler_to_quaternion(euler):
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]) 

def quaternion_to_euler(quaternion):
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def home_position():
  MS_cubic_home = [0, -2, 2, -1, 0, pi-2] #['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3', 'ee_fixed']
  group_arm.set_joint_value_target(MS_cubic_home)
  group_arm.go()
  
def scaling_plan(plan, scale):
  if scale > 1.0:
    print("too fast")
    sys.exit()
  
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

def move_global_system(distance, speed): #distanceはglobal座標系
  current_pose = get_current_pose()

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x + distance[0]
  target_pose.position.y = current_pose.position.y + distance[1]
  target_pose.position.z = current_pose.position.z + distance[2]
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0) #直線パスの生成
  scaling_plan(plan, speed)
  group_arm.execute(plan)
  return fraction #直線パスの成功率

def rotate(angle, speed):
  current_pose = get_current_pose()

  target_pose = Pose()
  target_pose.position = current_pose.position
  current_pose_euler = quaternion_to_euler(current_pose.orientation)
  current_pose_euler.x = current_pose_euler.x + angle[0]
  current_pose_euler.y = current_pose_euler.y + angle[1]
  current_pose_euler.z = current_pose_euler.z + angle[2]
  target_pose_quaternion = euler_to_quaternion(current_pose_euler)
  target_pose.orientation.x = target_pose_quaternion.x
  target_pose.orientation.y = target_pose_quaternion.y
  target_pose.orientation.z = target_pose_quaternion.z
  target_pose.orientation.w = target_pose_quaternion.w

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  scaling_plan(plan, speed)
  group_arm.execute(plan)
  return fraction #直線パスの成功率

def move_drill_system(distance, speed): #手先座標
  current_pose = get_current_pose()

  #手先座標系での目標位置と姿勢を定義
  goal_position_hand = [distance[0], distance[1], distance[2]] 
  goal_orientation_hand = [0.0, 0.0, 0.0, 1.0]  #手先座標系の姿勢は変わらない

  #現在のエンドエフェクタの位置と姿勢を抽出
  current_position = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
  current_orientation = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]

  #現在のエンドエフェクタの変換行列 (グローバル座標系)
  current_transformation = concatenate_matrices(translation_matrix(current_position), quaternion_matrix(current_orientation))
    
  #手先座標系での目標位置と姿勢の変換行列
  goal_transformation_hand = concatenate_matrices(translation_matrix(goal_position_hand), quaternion_matrix(goal_orientation_hand))
    
  #グローバル座標系への変換
  goal_transformation_global = concatenate_matrices(current_transformation, goal_transformation_hand)
    
  #目標位置と姿勢をグローバル座標系に変換
  goal_position_global = goal_transformation_global[:3, 3]
  goal_orientation_global = tf.transformations.quaternion_from_matrix(goal_transformation_global)
    
  #geometry_msgs.msg.Pose形式に変更
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
  scaling_plan(plan, speed)
  group_arm.execute(plan)
  return fraction #直線パスの成功率

def move_until_touch():
  f_absolute = calculate_force_absolute(f_current)
  while f_absolute < f_threshold:
    is_moved = move_drill_system([0.001, 0, 0], 0.02)
    f_absolute = calculate_force_absolute(f_current)
    if not is_moved:
      print("Error during moveing")
      sys.exit()
  return True

def angle_rotate_find(pitch_or_yaw):
  #使う成分の指定(0:x, 1:y, 2:z)
  if pitch_or_yaw == "pitch":
    r = 2
  elif pitch_or_yaw == "yaw":
    r = 1  
  else:
    print("Error setting pitch or yaw")
    sys.exit()
  
  f = [0, 0]
  angle = [0, 0, 0]
  f[0] = calculate_force_ratio([x - y for x, y in zip(f_filtered, initial_force)])
  f[1] = f[0]
  if f[0] > 0.1:
    angle[r] = 0.1
  elif f[0] < 0.1:
    angle[r] = 0.1
  rotate(angle, 0.02)

  while f[1] > 0.1:
    f[0] = f[1]
    f[1] = calculate_force_ratio([x - y for x, y in zip(f_filtered, initial_force)])
    if f[0] > f[1]:
      angle[r] = 0.1
    else:
      angle[r] = -0.1
    rotate(angle, 0.02)

  #print("Finish correction ", pitch_or_yaw)
  






def angle_correction(pitch_or_yaw, η): #逐次補正のコア部分
  #使う成分の指定(0:x, 1:y, 2:z)
  if pitch_or_yaw == "pitch":
    r = 2
  elif pitch_or_yaw == "yaw":
    r = 1  
  else:
    print("Error setting pitch or yaw")
    sys.exit()

  #初回の補正
  f0 = calculate_force_ratio([x - y for x, y in zip(f_filtered, initial_force)])
  Δθ1 = math.degrees(math.atan(f0[r]/f0[0])) #単位は度
  #print("Δθ1=",Δθ1)
  angle = [0,0,0]
  angle[r] = Δθ1
  rotate(angle, 0.02)

  #2回目以降の補正
  f1 = f0
  Δθi = Δθ1
  while Δθi > 0.5:
    f0 = f1
    f1 = calculate_force_ratio([x - y for x, y in zip(f_filtered, initial_force)])
    Δθi = Δθi * f1[r] * η / (f1[r] - f0[r])
    angle[r] = Δθi
    #print("Δθi=",Δθi)
    rotate(angle, 0.02)

  #print("Finish correction ", pitch_or_yaw)

def angle_integration(pitch_or_yaw, n): #統合補正のコア部分
  #使う成分の指定(0:x, 1:y, 2:z)
  if pitch_or_yaw == "pitch":
    r = 2
  elif pitch_or_yaw == "yaw":
    r = 1  
  else:
    print("Error setting pitch or yaw")
    sys.exit()

  
  angle = [0,0,0]
  Δθ = []
  ΔΦ = []
  #初回の補正
  f0 = calculate_force_ratio([x - y for x, y in zip(f_filtered, initial_force)])
  ΔΦ[0] = math.degrees(-1 * math.atan(f0[r]/f0[0])) #単位は度
  #print("Δθ1=",Δθ[0])
  Δθ[0] = ΔΦ[0] + 1
  angle[r] = Δθ
  rotate(angle, 0.02)

  #2回目以降の補正
  for i in range(n):
    fi = calculate_force_ratio([x - y for x, y in zip(f_filtered, initial_force)])
    ΔΦ[i+1] = math.degrees(-1 * math.atan(fi[r]/fi[0])) #単位は度
    #print("Δθi=",Δθ[i+1])
    Δθ[i+1] = ΔΦ[i+1] + 1
    angle[r] = Δθ[i+1]
    rotate(angle, 0.02)

  #最終的な補正値の決定
  Δθ_mean = (sum(Δθ) + sum(ΔΦ))/(n+1)
  angle[r] = Δθ_mean - sum(Δθ)
  rotate(angle, 0.02)


  #print("Finish correction ", pitch_or_yaw)




def angle_correction_experiment():
  angle_correction("pitch", 0.5)

def drill_experiment():
  home_position()
  global initial_force
  initial_force = f_filtered
  move_drill_system([0.1, 0.0, 0.0], 0.02)


def execute_experiment():
  global initial_force
  home_position()
  move_global_system([0.5, 0.0, 0.0], 0.5)
  initial_force = f_filtered
  print("Set initial")

  experiment_type = input("input experiment type a or d")
  if experiment_type == "a":
    angle_correction_experiment()
  elif experiment_type == "d":
    drill_experiment()

  else:
    print("Input correct type")
    sys.exit()



if __name__ == '__main__':
  try:
    rospy.init_node('drillanglecorrection')
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander("ur5_arm") #実機では名が違う

    group_arm.set_max_velocity_scaling_factor(1.0)

    # 力センサのトピックをSubscribeするコールバック関数を設定
    rospy.Subscriber('ft_sensor/raw', WrenchStamped, force_sensor_callback) #実機ではtopic名が違う

    # ロボットの制御を行うスレッドを開始
    control_thread = threading.Thread(target=execute_experiment())
    control_thread.start()

    rospy.spin()

  except rospy.ROSInterruptException:
    pass

