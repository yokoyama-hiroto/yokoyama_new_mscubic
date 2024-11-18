#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
from math import pi
from geometry_msgs.msg import WrenchStamped



def force_sensor_callback(msg):
    global force_sensor_value_x
    global force_sensor_value_y
    global force_sensor_value_z
    force_sensor_value_x = msg.wrench.force.x
    force_sensor_value_y = msg.wrench.force.y
    force_sensor_value_z = msg.wrench.force.z
    print("\r" + f'{force_sensor_value_x:6.1f}' +" " +  f'{force_sensor_value_y:6.1f}' + " " +  f'{force_sensor_value_z:6.1f}', end="")




if __name__ == '__main__':
  try:
    rospy.init_node('force_sensor_subscribe_mscubic')
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander("ur5_arm") 

    group_arm.set_max_velocity_scaling_factor(1.0)

    # 力センサのトピックを購読し、コールバック関数を設定
    rospy.Subscriber('ft_sensor/raw', WrenchStamped, force_sensor_callback)

    rospy.spin()

  except rospy.ROSInterruptException:
    pass

