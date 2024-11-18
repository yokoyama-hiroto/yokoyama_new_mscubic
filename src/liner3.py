#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import numpy as np
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander.robot_trajectory import RobotTrajectory

def interpolate_waypoints(start_pose, end_pose, eef_step):
    waypoints = []
    distance = np.linalg.norm([
        end_pose.position.x - start_pose.position.x,
        end_pose.position.y - start_pose.position.y,
        end_pose.position.z - start_pose.position.z
    ])
    num_steps = int(distance / eef_step) + 1

    for i in range(num_steps):
        alpha = i / float(num_steps - 1)
        waypoint = Pose()
        waypoint.position.x = start_pose.position.x + alpha * (end_pose.position.x - start_pose.position.x)
        waypoint.position.y = start_pose.position.y + alpha * (end_pose.position.y - start_pose.position.y)
        waypoint.position.z = start_pose.position.z + alpha * (end_pose.position.z - start_pose.position.z)
        waypoint.orientation = start_pose.orientation
        waypoints.append(waypoint)

    return waypoints

def compute_joint_values(move_group, waypoint):
    move_group.set_pose_target(waypoint)
    plan = move_group.plan()
    if plan and plan[1] > 0:
        joint_values = plan[1].joint_trajectory.points[-1].positions
        return joint_values
    return None

def check_collision(move_group, waypoint):
    move_group.set_pose_target(waypoint)
    plan = move_group.plan()
    return plan[1] > 0

def generate_robot_trajectory(joint_trajectory, move_group):
    trajectory = JointTrajectory()
    trajectory.joint_names = move_group.get_active_joints()

    for idx, joint_values in enumerate(joint_trajectory):
        point = JointTrajectoryPoint()
        point.positions = joint_values
        point.time_from_start = rospy.Duration(idx * 0.1)
        trajectory.points.append(point)

    robot_trajectory = RobotTrajectory()
    robot_trajectory.joint_trajectory = trajectory

    return robot_trajectory

def execute_trajectory(move_group, trajectory):
    success = move_group.execute(trajectory, wait=True)
    return success

def main():
    rospy.init_node('cartesian_path_example')
    moveit_commander.roscpp_initialize(sys.argv)

    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group_name = "ur5_arm"
    move_group = MoveGroupCommander(group_name)

    start_pose = move_group.get_current_pose().pose

    target_pose = Pose()
    target_pose.position.x = start_pose.position.x + 0.2
    target_pose.position.y = start_pose.position.y
    target_pose.position.z = start_pose.position.z
    target_pose.orientation = start_pose.orientation

    eef_step = 0.01
    waypoints = interpolate_waypoints(start_pose, target_pose, eef_step)

    joint_trajectory = []
    for waypoint in waypoints:
        joint_values = compute_joint_values(move_group, waypoint)
        if joint_values:
            joint_trajectory.append(joint_values)
        else:
            rospy.logwarn("逆運動学の計算に失敗しました")

    collision_free_trajectory = []
    for joint_values in joint_trajectory:
        if check_collision(move_group, joint_values):
            collision_free_trajectory.append(joint_values)
        else:
            rospy.logwarn("衝突が検出されました")

    robot_trajectory = generate_robot_trajectory(collision_free_trajectory, move_group)
    execute_trajectory(move_group, robot_trajectory)

if __name__ == "__main__":
    main()
