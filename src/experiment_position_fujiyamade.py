#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2020 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import moveit_commander
import math
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Point, Wrench, Vector3
from moveit_msgs.msg import Constraints, OrientationConstraint,PositionConstraint
import rosnode
from tf.transformations import quaternion_from_euler
import message_filters
import numpy as np
class PositionControl:
    def __init__(self):
        self.control_signal = 0.0
        self.kp=0.0
        #self.kd=0.0
        self.ki=0.0
        self.preforce=0.0
        self.error_integral=0.0

    def set_position(self, center_position=Point(0.2, 0.0, 0.1)): #Point(0.20, 0.0, 0.3)Point(0.25, 0.0, 0.2)Point(0.30, 0.0, 0.1)
        current_pos=arm.get_current_pose(end_effector_link=arm.get_end_effector_link())
        #print(current_pos)
        eef_step=0.01
        jump_threshold=0.0
        avoid_collisions=True
        way_points = []
        q = quaternion_from_euler(0.0, math.radians(90), 0.0)
        target_orientation = Quaternion(q[0], q[1], q[2], q[3])
        target_pose = Pose()
        kp=0.05
        self.control_signal+=(current_pos.pose.position.x-center_position.x)*kp
        target_pose.position.x = center_position.x-self.control_signal
        #target_pose.position.x = center_position.x
        target_pose.position.y = center_position.y
        target_pose.position.z = center_position.z
        target_pose.orientation = target_orientation
        print(target_pose.position.x)
        # arm.set_pose_target(target_pose)
        # arm.go()
        way_points.append(target_pose)
        path, fraction = arm.compute_cartesian_path(way_points, eef_step, jump_threshold, avoid_collisions)
        arm.execute(path)

    # def set_position_feedback(self):
    #     current_pos=arm.get_current_pose(self, end_effector_link=arm.get_end_effector_link())
    #     # print("jacobian")
    #     # print(type(jacobian))
    #     # print(jacobian)
    #     # print("torque")
    #     # print(type(torque))
    #     # print(torque)
    #     Torque = []
    #     for i in range(1, 8):##forceを行列に直す
    #         torque_x = eval('fs{}.wrench.torque.x'.format(i))
    #         torque_y = eval('fs{}.wrench.torque.y'.format(i))
    #         torque_z = eval('fs{}.wrench.torque.z'.format(i))
    #         Torque.append([torque_x, torque_y, torque_z])
    #     torque_norm = np.linalg.norm(Torque, axis=1)
    #     # print("torque_norm")
    #     # print(torque_norm)
    #     # print("Torque")
    #     # print(Torque)
    #     estimate_force = np.dot(jacobian_inv.T,torque_norm)
    #     error = force[0].x-estimate_force[0]
    #     self.kp = 0.00005 #0.0005
    #     #self.kd=0.0001
    #     self.ki = 0.00000001 #0.0000001
    #     if error <= 5.0 and error >= -5.0:
    #         self.control_signal = 0.0
    #     # elif error >= 30.0:
    #     #     self.control_signal = 0.0
    #     elif error <=-3.0:
    #         # print(error)
    #         error+=3.0
    #         self.error_integral += error
    #         self.control_signal = self.kp * error + self.ki * self.error_integral
    #     else:
    #         # print(error)
    #         error-=5.0
    #         self.error_integral += error
    #         self.control_signal = self.kp * error + self.ki * self.error_integral
    #     # print("a")
    #     print("estimate_force")
    #     print(estimate_force)
    #     print("force")
    #     print(force[0].x)
    #     # print("control_signal")
    #     #print(self.control_signal)

def main():
    arm.set_named_target("home")
    arm.go()
    position_control=PositionControl()
    # Create a subscriber to listen to the joint_states topic
    # 座標(x=0.3, y=0.0, z=0.1)を中心に、YZ平面上に一辺0.1 mの正方形領域を3回走査するように手先を動かす
    # Create subscribers to listen to the ft_sensor_topic_1 to ft_sensor_topic_7 topics
    #rospy.Subscriber('ft_sensor_topic_1', WrenchStamped, force_control.set_force_feedback)
    # Create a TimeSynchronizer to synchronize the messages from all the subscribers
    # Register a callback function to handle the synchronized messages
    while (rospy.is_shutdown()==False):
        position_control.set_position()
        # Set Path Constraint
        constraints = Constraints()
        constraints.name = "constraint1"
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = arm.get_planning_frame()
        orientation_constraint.link_name = arm.get_end_effector_link()
        q = quaternion_from_euler(0.0, math.radians(90), 0.0)
        target_orientation = Quaternion(q[0], q[1], q[2], q[3])
        orientation_constraint.orientation = target_orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.18
        orientation_constraint.absolute_y_axis_tolerance = 0.18
        orientation_constraint.absolute_z_axis_tolerance = 0.18
        orientation_constraint.weight = 1.0
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = arm.get_planning_frame()
        position_constraint.link_name = arm.get_end_effector_link()
        position_constraint.target_point_offset.x = 0.5
        position_constraint.target_point_offset.y = 0.05
        position_constraint.target_point_offset.z = 0.05
        position_constraint.weight = 1.0
        constraints.orientation_constraints.append( orientation_constraint )
        constraints.position_constraints.append( position_constraint )
        arm.set_path_constraints( constraints )
        
    arm.set_named_target("vertical")
    arm.go()

    # Spin the ROS node to receive messages
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('square_path', anonymous=True)
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
