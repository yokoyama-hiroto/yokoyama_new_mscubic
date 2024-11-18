#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
from math import pi
import threading
from geometry_msgs.msg import Pose
from moveit_msgs.msg import JointLimits

scale = 0.001

def home(group_arm):
  MS_cubic_home = [0, -1.3, 1.3, -1.2, 0, 1.2-pi] #['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'ee_fixed_joint']
  group_arm.set_joint_value_target(MS_cubic_home)
  group_arm.go()

def set_speed_scale(scale):
  speed = [rospy.get_param("/robot_description_planning/joint_limits/shoulder_pan_joint/max_velocity"), rospy.get_param("/robot_description_planning/joint_limits/shoulder_lift_joint/max_velocity"), rospy.get_param("/robot_description_planning/joint_limits/elbow_joint/max_velocity"), rospy.get_param("/robot_description_planning/joint_limits/wrist_1_joint/max_velocity"), rospy.get_param("/robot_description_planning/joint_limits/wrist_2_joint/max_velocity"), rospy.get_param("/robot_description_planning/joint_limits/wrist_3_joint/max_velocity")]
  scaledspeed = [n*scale for n in speed]
  print(scaledspeed)
  rospy.set_param("/robot_description_planning/joint_limits/shoulder_pan_joint/max_velocity", scaledspeed[0])
  rospy.set_param("/robot_description_planning/joint_limits/shoulder_lift_joint/max_velocity", scaledspeed[1])
  rospy.set_param("/robot_description_planning/joint_limits/elbow_joint/max_velocity", scaledspeed[2])
  rospy.set_param("/robot_description_planning/joint_limits/wrist_1_joint/max_velocity", scaledspeed[3])
  rospy.set_param("/robot_description_planning/joint_limits/wrist_2_joint/max_velocity", scaledspeed[4])
  rospy.set_param("/robot_description_planning/joint_limits/wrist_3_joint/max_velocity", scaledspeed[5])

def force_sensor_callback(msg): #センサー値を取得するコールバック関数
  print(msg)

def get_current_pose(group_arm): #現在のtcpの位置姿勢を取得
  current_pose = group_arm.get_current_pose("tcp").pose
  return current_pose

def move_X(group_arm, distance):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x + distance
  target_pose.position.y = current_pose.position.y
  target_pose.position.z = current_pose.position.z
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  (plan, fraction) = group_arm.compute_cartesian_path([current_pose, target_pose], 0.001, 0.0) #直線パスの生成
  group_arm.execute(plan)
  """
  #print(plan)
  #print("aaa")
  #print(dir(plan.joint_trajectory.points))
  #print(plan.joint_trajectory.points)
  print(len(plan.joint_trajectory.points))
  for i in range(len(plan.joint_trajectory.points)):
    print(i)
    print(plan.joint_trajectory.points[i].positions)
    group_arm.set_joint_value_target(plan.joint_trajectory.points[i].positions)
    group_arm.go()
"""
  





def mover(group_arm):
  #home(group_arm)
  rospy.sleep(1)
  
  move_X(group_arm, 0.3)




if __name__ == '__main__':
  try:
    rospy.init_node('move_group_interface_tutorial2')
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander("ur5_arm") 

    #group_arm.set_max_velocity_scaling_factor(1.0)
    #group_arm.set_max_acceleration_scaling_factor(1.0)
    #set_speed_scale(scale)

    #rospy.Subscriber('ft_sensor/raw', JointLimits, force_sensor_callback)

    # ロボットの制御を行うスレッドを開始
    control_thread = threading.Thread(target=mover(group_arm))
    control_thread.start()

    rospy.spin()

  except rospy.ROSInterruptException:
    pass

