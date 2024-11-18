#!/usr/bin/env python3
import rospy
import sys
import tf
import moveit_commander
from math import pi
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_matrix, translation_matrix, concatenate_matrices


sleep_time = 0.1 #sleeping time
distance = 0.5 #移動距離
angle = pi/9   #移動角度
home_pose = Pose()

def get_current_pose(group_arm): #現在のtcpの位置姿勢を取得
  current_pose = group_arm.get_current_pose("tcp").pose
  return current_pose

def home(group_arm):
  MS_cubic_home = [0, -2, 2, -1, 0, pi-2] #['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'ee_fixed_joint']
  group_arm.set_joint_value_target(MS_cubic_home)
  group_arm.go()
  return True

  
def move_X(group_arm):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x + distance
  target_pose.position.y = current_pose.position.y
  target_pose.position.z = current_pose.position.z
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0) #直線パスの生成
  ###########実行に問題があれば出力
  if fraction < 0.5:
    print("Error with computing path. fraction = {}".format(fraction))

  is_executed = group_arm.execute(plan)
  if not is_executed:
    print("Error on execution")
    
  return True #問題がないことを返す


def move_x(group_arm): #手先座標 -xがドリル軸進行方向
    distance = -0.1
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
    target_pose = Pose()
    target_pose.position.x = goal_position_global[0]
    target_pose.position.y = goal_position_global[1]
    target_pose.position.z = goal_position_global[2]
    target_pose.orientation.x = goal_orientation_global[0]
    target_pose.orientation.y = goal_orientation_global[1]
    target_pose.orientation.z = goal_orientation_global[2]
    target_pose.orientation.w = goal_orientation_global[3]

    group_arm.set_pose_target(target_pose)
    (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
    group_arm.execute(plan)








def mover(group_arm):
  #home(group_arm)
  move_X(group_arm)








if __name__ == '__main__':
  try:
    rospy.init_node('move_group_interface_tutorial2')
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander("ur5_arm") 

    group_arm.set_max_velocity_scaling_factor(1.0)
    group_arm.set_max_acceleration_scaling_factor(0.01)

    mover(group_arm)

    rospy.spin()

  except rospy.ROSInterruptException:
    pass

