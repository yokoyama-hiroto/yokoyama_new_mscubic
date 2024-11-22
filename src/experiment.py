#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import tf
import geometry_msgs.msg
import math
from math import pi
import threading
from moveit_commander import PlanningSceneInterface
from tf.transformations import quaternion_matrix, translation_matrix, concatenate_matrices
from geometry_msgs.msg import Pose, WrenchStamped, Vector3, Quaternion


k = 0.002/(0.002 + 1) #ローパスフィルタの係数
f_threshold = 5 #接触力
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
  print("force", f)
  f_absolute = math.sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2])
  return f_absolute

def calculate_force_ratio(f): #センサー値の成分比率を算出
  f_absolute = calculate_force_absolute(f)
  f_ratio = [n/f_absolute for n in f]
  return f_ratio

def get_current_pose(group_arm): #現在のtcpの位置姿勢を取得
  current_pose = group_arm.get_current_pose("tcp").pose #"tool0" or "tcp"
  return current_pose

def euler_to_quaternion(euler):
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]) 

def quaternion_to_euler(quaternion):
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def home_position(group_arm):
  MS_cubic_home = [0, -1.5, 1.5, 0, pi/2, -pi/2] #['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
  group_arm.set_joint_value_target(MS_cubic_home)
  group_arm.go()
  
def scaling_plan(plan, scale):
  if scale > 1:
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

def move_global_system(group_arm, distance, speed): #global座標系
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x + distance[0]
  target_pose.position.y = current_pose.position.y + distance[1]
  target_pose.position.z = current_pose.position.z + distance[2]
  target_pose.orientation = current_pose.orientation

  #group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.001, 0.0)
  scaling_plan(plan, speed)
  if flag_debug_mode:
    print(plan)
    print("fraction:", fraction)
  group_arm.execute(plan)
  return fraction #直線パスの成功率


def rotate(group_arm, angle, speed):
  current_pose = get_current_pose(group_arm)

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

  #group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  scaling_plan(plan, speed)
  if flag_debug_mode:
    print(plan)
    print("fraction:", fraction)
  group_arm.execute(plan)
  return fraction #直線パスの成功率

def move_drill_system(group_arm, distance, speed): #手先座標
  current_pose = get_current_pose(group_arm)

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

  #group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
  scaling_plan(plan, speed)
  if flag_debug_mode:
    print(plan)
    print("fraction:", fraction)
  group_arm.execute(plan)
  return fraction #直線パスの成功率

def move_until_touch(group_arm):
  rospy.sleep(0.01)
  f_absolute = calculate_force_absolute([x - y for x, y in zip(filtered_force_value, initial_force)])
  if flag_debug_mode:
    print(f_absolute)

  while f_absolute < f_threshold:
    is_moved = move_global_system(group_arm, [0, 0, -0.01], 1)
    f_absolute = calculate_force_absolute([x - y for x, y in zip(filtered_force_value, initial_force)])
    if flag_debug_mode:
      print(f_absolute)

    if not is_moved:
      print("Error before touch")
      sys.exit()
  return True



def angle_rotate_find(group_arm, pitch_or_yaw): #連続計測
  rospy.sleep(0.01)
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
  f[0] = calculate_force_ratio([x - y for x, y in zip(filtered_force_value, initial_force)])[r]
  f[1] = f[0]
  if f[0] > 0.1:
    angle[r] = 0.1
  elif f[0] < 0.1:
    angle[r] = 0.1
  print(angle)
  rotate(group_arm, angle, 0.02)
  rospy.sleep(0.01)

  while abs(f[1]) > 0.1:
    f[0] = f[1]
    f[1] = calculate_force_ratio([x - y for x, y in zip(filtered_force_value, initial_force)])[r]
    if f[0] > f[1]:
      angle[r] = 0.1
    else:
      angle[r] = -0.1
    print(angle)
    rotate(group_arm, angle, 0.02)
    rospy.sleep(0.01)

  print("Finish correction ", pitch_or_yaw)
  
def angle_correction_b(group_arm, pitch_or_yaw, η): #逐次補正
  rospy.sleep(0.01)
  #使う成分の指定(0:x, 1:y, 2:z)
  if pitch_or_yaw == "pitch":
    r = 2
  elif pitch_or_yaw == "yaw":
    r = 1  
  else:
    print("Error setting pitch or yaw")
    sys.exit()

  #初回の補正
  f0 = calculate_force_ratio([x - y for x, y in zip(filtered_force_value, initial_force)])
  Δθ1 = math.degrees(math.atan(f0[r]/f0[0])) #単位は度
  #print("Δθ1=",Δθ1)
  angle = [0,0,0]
  angle[r] = Δθ1
  print(angle)
  rotate(group_arm, angle, 0.02)
  rospy.sleep(0.01)

  #2回目以降の補正
  f1 = f0
  Δθi = Δθ1
  while Δθi > 0.5:
    f0 = f1
    f1 = calculate_force_ratio([x - y for x, y in zip(filtered_force_value, initial_force)])
    Δθi = Δθi * f1[r] * η / (f1[r] - f0[r])
    angle[r] = Δθi
    print(angle)
    rotate(group_arm, angle, 0.02)
    rospy.sleep(0.01)

  #print("Finish correction ", pitch_or_yaw)

def angle_integration(group_arm, pitch_or_yaw, n): #統合補正
  rospy.sleep(0.01)
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
  f0 = calculate_force_ratio([x - y for x, y in zip(filtered_force_value, initial_force)])
  ΔΦ.append(math.degrees(-1 * math.atan(f0[r]/f0[0]))) #単位は度
  #print("Δθ1=",Δθ[0])
  Δθ.append(ΔΦ[0])
  angle[r] = Δθ[0]
  print(angle)
  rotate(group_arm, angle, 0.02)
  rospy.sleep(0.01)

  #2回目以降の補正
  for i in range(n):
    fi = calculate_force_ratio([x - y for x, y in zip(filtered_force_value, initial_force)])
    ΔΦ.append(math.degrees(-1 * math.atan(fi[r]/fi[0]))) #単位は度
    #print("Δθi=",Δθ[i+1])
    Δθ.append(ΔΦ[i+1])
    angle[r] = Δθ[i+1]
    print(angle)
    rotate(group_arm, angle, 0.02)
    rospy.sleep(0.01)

  #最終的な補正値の決定
  Δθ_mean = (sum(Δθ) + sum(ΔΦ))/(n+1)
  angle[r] = Δθ_mean - sum(Δθ)
  print("angle",angle)
  rotate(group_arm, angle, 0.02)
  rospy.sleep(0.01)


  #print("Finish correction ", pitch_or_yaw)




def angle_correction_experiment(group_arm):
  move_until_touch(group_arm)
  angle_rotate_find(group_arm, "pitch")
  #angle_correction_b(group_arm, "pitch", 0.5)
  #angle_integration(group_arm, "pitch", 4)

def drill_experiment(group_arm):
  move_until_touch(group_arm)
  angle_rotate_find(group_arm, "pitch", 0.02)


def execute_experiment(group_arm):
  home_position(group_arm)
  #rotate(group_arm, [0, pi/9, 0], 1) #0,1,3,5->0,pi/180,pi/60,pi/36初期入射角設定
  
  setting_initial = move_global_system(group_arm, [0.0, 0.0, -0.2], 1) #加工位置決定
  global initial_force
  initial_force = list(filtered_force_value)
  move_until_touch(group_arm)

  if flag_debug_mode:
    setting_initial = 1
  
  rospy.sleep(1)
  if setting_initial > 0.8:
    initial_force = filtered_force_value
    print("Set initial")
    experiment_type = input("experiment type (a:angle correction or d:drilling)")
    if experiment_type == "a":
      angle_correction_experiment(group_arm)
    elif experiment_type == "d":
      drill_experiment(group_arm)
    else:
      print("Input correct type")
      sys.exit()
  else:
    print(setting_initial)
    print("Error during setting initial")
    sys.exit()



if __name__ == '__main__':
  try:
    sim_or_real = input("1:simulation or 2:real?")
    if sim_or_real == "1":
      ur5_arm = "ur5_arm"
      rostopic_force = "ft_sensor/raw"
    elif sim_or_real == "2":
      ur5_arm ="manipulator"
      rostopic_force = "wrench"
    else:
      print("Input 1 or 2")
      sys.exit()

    global flag_debug_mode
    flag_debug_mode = False

    rospy.init_node('DrillAngleCorrection')
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander(ur5_arm)

    group_arm.set_max_velocity_scaling_factor(1.0)

    # 力センサのトピックをSubscribeするコールバック関数
    rospy.Subscriber(rostopic_force, WrenchStamped, force_sensor_callback)

    # ロボット制御を行うスレッドを開始
    control_thread = threading.Thread(target=execute_experiment(group_arm))
    control_thread.start()



    rospy.spin()

  except rospy.ROSInterruptException:
    pass

