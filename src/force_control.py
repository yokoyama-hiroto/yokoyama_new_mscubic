#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import time
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose
from geometry_msgs.msg import WrenchStamped

threshold = 20.0 #threshold value
sleep_time = 0.1 #sleeping time
distance = 0.001


def wrench_callback(data):
  sys.stdout.write("\rFx = {:4.1f}, Fy = {:4.1f}, Fz = {:4.1f}, Tx = {:4.1f}, Ty = {:4.1f}, Tz = {:4.1f}".format(data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z))
  sys.stdout.flush()

def wrench_z(data):
  Fz = data.wrench.force.z * (-1)
  if(Fz > threshold):
    move_up(group_arm)
  else:
    move_down(group_arm)

def get_current_pose(group_arm):
  current_pose = group_arm.get_current_pose("ee_link").pose
  return current_pose

def move_up(group_arm):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x
  target_pose.position.y = current_pose.position.y
  target_pose.position.z = current_pose.position.z + distance
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  group_arm.go()

def move_down(group_arm):
  current_pose = get_current_pose(group_arm)

  target_pose = Pose()
  target_pose.position.x = current_pose.position.x
  target_pose.position.y = current_pose.position.y
  target_pose.position.z = current_pose.position.z - distance
  target_pose.orientation = current_pose.orientation

  group_arm.set_pose_target(target_pose)
  group_arm.go()

def listener():
  rospy.Subscriber("ft_sensor/raw", WrenchStamped, wrench_callback)

def mover():
  rospy.Subscriber("ft_sensor/raw", WrenchStamped, wrench_z)
  



if __name__ == '__main__':
  try:
    rospy.init_node('move_group_interface_tutorial2')
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander("ur5_arm")

    group_arm.set_max_velocity_scaling_factor(0.001)

    listener()
    #mover()

    rospy.spin()

  except rospy.ROSInterruptException:
    pass

