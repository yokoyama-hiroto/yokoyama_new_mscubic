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
from tf.transformations import quaternion_matrix, quaternion_multiply, translation_matrix, concatenate_matrices
from geometry_msgs.msg import Pose, WrenchStamped, Vector3, Quaternion
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint


k = 0.002/(0.002 + 1) #ローパスフィルタの係数
f_threshold = 10 #接触力
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
sleep_time = 0.01
correction_speed = 0.5

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
  if flag_debug_mode:
    print("force before calculate absol", f)
  f_absolute = math.sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2])
  return f_absolute

def calculate_force_ratio(f): #センサー値の成分比率を算出
  f_absolute = calculate_force_absolute(f)
  f_ratio = [n/f_absolute for n in f]
  return f_ratio

def get_current_pose(group_arm): #現在のtcpの位置姿勢を取得
  current_pose = group_arm.get_current_pose(group_arm.get_end_effector_link()).pose #"tool0" or "tcp"
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
    # print(plan)
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
  #print([target_pose.position.x-current_pose.position.x, target_pose.position.y-current_pose.position.y, target_pose.position.z-current_pose.position.z])
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.001, 0.0)
  scaling_plan(plan, speed)
  if flag_debug_mode:
    # print(plan)
    print("fraction:", fraction)
  group_arm.execute(plan)
  return fraction #直線パスの成功率

def move_global_system_constraints(group_arm, distance, speed): #global座標系cartesianpathを使わない
  current_pose = get_current_pose(group_arm)
  
  constraints = Constraints()
  constraints.name = "move_global_system"
  orientation_constraint = OrientationConstraint()
  orientation_constraint.header.frame_id = group_arm.get_planning_frame()
  orientation_constraint.link_name = group_arm.get_end_effector_link()
  orientation_constraint.orientation = current_pose.orientation
  orientation_constraint.absolute_x_axis_tolerance = 0.001
  orientation_constraint.absolute_y_axis_tolerance = 0.001
  orientation_constraint.absolute_z_axis_tolerance = 0.001
  orientation_constraint.weight = 1.0
  position_constraint = PositionConstraint()
  position_constraint.header.frame_id = group_arm.get_planning_frame()
  position_constraint.link_name = group_arm.get_end_effector_link()
  position_constraint.target_point_offset.x = 0.0001
  position_constraint.target_point_offset.y = 0.0001
  position_constraint.target_point_offset.z = 0.0001
  position_constraint.weight = 1.0
  constraints.orientation_constraints.append( orientation_constraint )
  constraints.position_constraints.append( position_constraint )
  group_arm.set_path_constraints( constraints )

  target_pose = [0, 0, 0]
  target_pose[0] = current_pose.position.x + distance[0]
  target_pose[1] = current_pose.position.y + distance[1]
  target_pose[2] = current_pose.position.z + distance[2]

  group_arm.set_max_velocity_scaling_factor(speed)
  group_arm.set_max_acceleration_scaling_factor(1.0)

  group_arm.set_position_target(target_pose)
  result = group_arm.go()
  if flag_debug_mode:
    print("group_arm.go", result)
  return result

def move_drill_system_constraints(group_arm, distance, speed): #ドリル座標系cartesianpathを使わない
  current_pose = get_current_pose(group_arm)
  
  constraints = Constraints()
  constraints.name = "move_drill_system"
  orientation_constraint = OrientationConstraint()
  orientation_constraint.header.frame_id = group_arm.get_planning_frame()
  orientation_constraint.link_name = group_arm.get_end_effector_link()
  orientation_constraint.orientation = current_pose.orientation
  orientation_constraint.absolute_x_axis_tolerance = 0.001
  orientation_constraint.absolute_y_axis_tolerance = 0.001
  orientation_constraint.absolute_z_axis_tolerance = 0.001
  orientation_constraint.weight = 1.0
  position_constraint = PositionConstraint()
  position_constraint.header.frame_id = group_arm.get_planning_frame()
  position_constraint.link_name = group_arm.get_end_effector_link()
  position_constraint.target_point_offset.x = 0.0001
  position_constraint.target_point_offset.y = 0.0001
  position_constraint.target_point_offset.z = 0.0001
  position_constraint.weight = 1.0
  constraints.orientation_constraints.append( orientation_constraint )
  constraints.position_constraints.append( position_constraint )
  group_arm.set_path_constraints( constraints )

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
    
  #geometry_msgs.msg.Pose形式に変更
  target_pose = [0, 0, 0]
  target_pose[0] = goal_position_global[0]
  target_pose[1] = goal_position_global[1]
  target_pose[2] = goal_position_global[2]

  print("target_position", target_pose)

  group_arm.set_max_velocity_scaling_factor(speed)
  group_arm.set_max_acceleration_scaling_factor(1.0)

  group_arm.set_position_target(target_pose)
  result = group_arm.go()
  print("group_arm.go", result)
  return result

"""
def rotate_euler(group_arm, angle, speed):#euler角の特異点では軌道がおかしくなる
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position = current_pose.position
  current_pose_euler = quaternion_to_euler(current_pose.orientation)
  current_pose_euler.x = current_pose_euler.x + angle[0]
  current_pose_euler.y = current_pose_euler.y + angle[1]
  current_pose_euler.z = current_pose_euler.z + angle[2]
  #print(current_pose_euler)
  target_pose_quaternion = euler_to_quaternion(current_pose_euler)
  target_pose.orientation.x = target_pose_quaternion.x
  target_pose.orientation.y = target_pose_quaternion.y
  target_pose.orientation.z = target_pose_quaternion.z
  target_pose.orientation.w = target_pose_quaternion.w
  #print(target_pose.orientation)

  #group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.001, 0.0)
  scaling_plan(plan, speed)
  if flag_debug_mode:
    #print(plan)
    print("fraction:", fraction)
  group_arm.execute(plan)
  return fraction #直線パスの成功率
"""

def rotate(group_arm, angle, speed):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position = current_pose.position

  angle_quaternion = tf.transformations.quaternion_from_euler(angle[0], angle[1], angle[2])

  current_quaternion = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
  target_pose_quaternion = quaternion_multiply(current_quaternion, angle_quaternion)
  target_pose.orientation.x = target_pose_quaternion[0]
  target_pose.orientation.y = target_pose_quaternion[1]
  target_pose.orientation.z = target_pose_quaternion[2]
  target_pose.orientation.w = target_pose_quaternion[3]

  #group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.001, 0.0)
  scaling_plan(plan, speed)
  if flag_debug_mode:
    #print(plan)
    print("fraction:", fraction)
  group_arm.execute(plan)
  return fraction #直線パスの成功率

def move_until_touch(group_arm):
  rospy.sleep(sleep_time)
  f_absolute = calculate_force_absolute([x - y for x, y in zip(filtered_force_value, initial_force)])
  if flag_debug_mode:
    print("move until touch")
    print("filtered_force", filtered_force_value)
    print("initial_force", initial_force)
    print("absolute", f_absolute)

  while f_absolute < f_threshold:
    is_moved = move_drill_system(group_arm, [-0.001, 0.0, 0.0], correction_speed)
    f_absolute = calculate_force_absolute([x - y for x, y in zip(filtered_force_value, initial_force)])
    if flag_debug_mode:
      print("filtered_force", filtered_force_value)
      print("initial_force", initial_force)
      print("absolute",f_absolute)

    if not is_moved:
      print("Error before touch")
      sys.exit()

  print("Touch!")
  return True


def angle_rotate_find(group_arm, pitch_or_yaw): #連続計測
  rospy.sleep(sleep_time)
  #使う成分の指定(0:x, 1:y, 2:z)
  if pitch_or_yaw == "pitch":
    r = 2
    a = 1
  elif pitch_or_yaw == "yaw":
    r = 1 
    a = 2 
  else:
    print("Error setting pitch or yaw")
    sys.exit()
  global initial_force

  f = [0, 0]
  angle = [0, 0, 0]
  f[0] = calculate_force_ratio([x - y for x, y in zip(filtered_force_value, initial_force)])[r]
  f[1] = f[0]
  if f[0] > 0.1:
    angle[a] = pi/1800
  elif f[0] < 0.1:
    angle[a] = -pi/1800
  print("rotate_angle", angle)
  move_drill_system(group_arm, [0.01, 0, 0], correction_speed)
  initial_force = list(filtered_force_value)
  rotate(group_arm, angle, correction_speed)
  move_until_touch(group_arm)
  rospy.sleep(sleep_time)

  pid = [20, 1, 0]
  error_p_previous = 0
  error_i = 0



  while abs(f[1]) > 0.01:
    f[0] = f[1]
    f[1] = calculate_force_ratio([x - y for x, y in zip(filtered_force_value, initial_force)])[r]
    error_p = f[1]
    error_i = error_i + (error_p_previous + error_p) * sleep_time / 2
    error_d = f[1] - error_p_previous/sleep_time

    u = pid[0]*error_p + pid[1]*error_i + pid[2]*error_d
    angle[a] = -pi/1800 * u
    error_p_previous = error_p
    print("rotate_angle",angle)
    move_drill_system(group_arm, [0.01, 0, 0], correction_speed)
    initial_force = list(filtered_force_value)
    rotate(group_arm, angle, correction_speed)
    move_until_touch(group_arm)
    print("u", u)
    print("rotate angle", angle)
    print("abs", abs(f[1]))
    rospy.sleep(sleep_time)

  print("Finish correctionA ", pitch_or_yaw)
  print("Final Force:", [x - y for x, y in zip(filtered_force_value, initial_force)])
  current_pose = get_current_pose(group_arm)
  current_pose_euler = quaternion_to_euler(current_pose.orientation)
  print("Final pose", current_pose_euler)
  
def angle_correction_b(group_arm, pitch_or_yaw, η): #逐次補正
  rospy.sleep(sleep_time)
  #使う成分の指定(0:x, 1:y, 2:z)
  if pitch_or_yaw == "pitch":
    r = 2
    a = 1
  elif pitch_or_yaw == "yaw":
    r = 1
    a= 2  
  else:
    print("Error setting pitch or yaw")
    sys.exit()

  global initial_force
  #初回の補正
  f0 = calculate_force_ratio([x - y for x, y in zip(filtered_force_value, initial_force)])
  Δθ1 = -1 * math.atan(f0[r]/f0[0]) #radian
  #print("Δθ1=",Δθ1)
  angle = [0,0,0]
  angle[a] = Δθ1
  print("rotate_angle",angle)
  move_drill_system(group_arm, [0.01, 0, 0], correction_speed)
  initial_force = list(filtered_force_value)
  rotate(group_arm, angle, correction_speed)
  move_until_touch(group_arm)
  rospy.sleep(sleep_time)

  #2回目以降の補正
  f1 = f0
  Δθi = Δθ1
  while Δθi > pi/360:
    f0 = f1
    f1 = calculate_force_ratio([x - y for x, y in zip(filtered_force_value, initial_force)])
    Δθi = Δθi * f1[r] * η / (f1[r] - f0[r])
    angle[a] = Δθi
    print("rotate_angle", angle)
    move_drill_system(group_arm, [0.01, 0, 0], correction_speed)
    initial_force = list(filtered_force_value)
    rotate(group_arm, angle, correction_speed)
    move_until_touch(group_arm)
    rospy.sleep(sleep_time)

  print("Finish correctionB ", pitch_or_yaw)
  print("Final Force:", [x - y for x, y in zip(filtered_force_value, initial_force)])
  current_pose = get_current_pose(group_arm)
  current_pose_euler = quaternion_to_euler(current_pose.orientation)
  print("Final pose", current_pose_euler)

def angle_integration(group_arm, pitch_or_yaw, n): #統合補正
  rospy.sleep(sleep_time)
  #使う成分の指定(0:x, 1:y, 2:z)
  if pitch_or_yaw == "pitch":
    r = 2
    a = 1
  elif pitch_or_yaw == "yaw":
    r = 1  
    a = 2
  else:
    print("Error setting pitch or yaw")
    sys.exit()

  global initial_force
  
  angle = [0,0,0]
  Δθ = []
  ΔΦ = []
  #初回の補正
  f0 = calculate_force_ratio([x - y for x, y in zip(filtered_force_value, initial_force)])
  ΔΦ.append(-1 * math.atan(f0[r]/f0[0])) #単位はradian
  #print("Δθ1=",Δθ[0])
  Δθ.append(ΔΦ[0] * 2)
  angle[a] = Δθ[0]
  print("rotate_angle", angle)
  move_drill_system(group_arm, [0.01, 0, 0], correction_speed)
  initial_force = list(filtered_force_value)
  rotate(group_arm, angle, correction_speed)
  move_until_touch(group_arm)
  rospy.sleep(sleep_time)

  #2回目以降の補正
  for i in range(n):
    fi = calculate_force_ratio([x - y for x, y in zip(filtered_force_value, initial_force)])
    ΔΦ.append(-1 * math.atan(fi[r]/fi[0])) #単位はradian
    #print("Δθi=",Δθ[i+1])
    Δθ.append(ΔΦ[i+1] * 2)
    angle[a] = Δθ[i+1]
    print("rotate_angle", angle)
    move_drill_system(group_arm, [0.01, 0, 0], correction_speed)
    initial_force = list(filtered_force_value)
    rotate(group_arm, angle, correction_speed)
    move_until_touch(group_arm)
    rospy.sleep(sleep_time)

  #最終的な補正値の決定
  Δθ_mean = (sum(Δθ) + sum(ΔΦ))/(n+1)
  angle[a] = Δθ_mean - sum(Δθ)
  print("Last_angle_mean", angle)
  move_drill_system(group_arm, [0.01, 0, 0], correction_speed)
  initial_force = list(filtered_force_value)
  rotate(group_arm, angle, correction_speed)
  move_until_touch(group_arm)
  rospy.sleep(sleep_time)

  print("Finish correctionC ", pitch_or_yaw)
  print("Final Force:", [x - y for x, y in zip(filtered_force_value, initial_force)])
  current_pose = get_current_pose(group_arm)
  current_pose_euler = quaternion_to_euler(current_pose.orientation)
  print("Final pose", current_pose_euler)




def angle_correction_experiment(group_arm):
  move_until_touch(group_arm)
  # angle_rotate_find(group_arm, "pitch")
  # angle_rotate_find(group_arm, "yaw") #yaw方向の回転自体がおかしい？要確認
  angle_correction_b(group_arm, "pitch", 0.5)
  # angle_integration(group_arm, "pitch", 4)

  sys.exit()

def drill_experiment(group_arm):
  move_until_touch(group_arm)
  angle_rotate_find(group_arm, "pitch")


def execute_experiment(group_arm):
  home_position(group_arm)
  # rotate(group_arm, [0, pi/9, 0], 1) #0,1,3,5->0,pi/180,pi/60,pi/36初期入射角設定
  # move_global_system_constraints(group_arm, [0, 0, -0.13], 1)
  
  setting_initial = move_global_system(group_arm, [0.0, 0.0, -0.14], 1) #加工位置決定
  global initial_force
  initial_force = list(filtered_force_value)
  
  rospy.sleep(1)
  if setting_initial > 0.8:
    print("Set initial")
    angle_correction_experiment(group_arm)#
    experiment_type = input("experiment type (a:angle correction or d:drilling)")
    if experiment_type == "a":
      angle_correction_experiment(group_arm)
    elif experiment_type == "d":
      drill_experiment(group_arm)
    else:
      print("Input correct type")
      sys.exit()
  else:
    print("setting_initial_fraction", setting_initial)
    print("Error during setting initial")
    sys.exit()



if __name__ == '__main__':
  try:
    global flag_debug_mode
    flag_debug_mode = False
    if not flag_debug_mode:
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
    else:
      ur5_arm = "ur5_arm"
      rostopic_force = "ft_sensor/raw" 

    rospy.init_node('DrillAngleCorrection')
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander(ur5_arm)

    group_arm.set_max_velocity_scaling_factor(1.0)
    group_arm.set_max_acceleration_scaling_factor(1.0)

    # 力センサのトピックをSubscribeするコールバック関数
    rospy.Subscriber(rostopic_force, WrenchStamped, force_sensor_callback)

    # ロボット制御を行うスレッドを開始
    control_thread = threading.Thread(target=execute_experiment(group_arm))
    control_thread.start()

    rospy.spin()

  except rospy.ROSInterruptException:
    pass

