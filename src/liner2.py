#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import numpy as np
from geometry_msgs.msg import Pose
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface

def interpolate_waypoints(start_pose, end_pose, step_size):
    """
    エンドエフェクタの位置間の直線経路を補間する

    :param start_pose: 現在のエンドエフェクタの位置
    :param end_pose: 目標位置
    :param step_size: ステップサイズ
    :return: 補間された位置のリスト
    """
    waypoints = []
    distance = np.linalg.norm([
        end_pose.position.x - start_pose.position.x,
        end_pose.position.y - start_pose.position.y,
        end_pose.position.z - start_pose.position.z
    ])
    num_steps = int(distance / step_size) + 1

    for i in range(num_steps):
        alpha = i / float(num_steps - 1)
        waypoint = Pose()
        waypoint.position.x = start_pose.position.x + alpha * (end_pose.position.x - start_pose.position.x)
        waypoint.position.y = start_pose.position.y + alpha * (end_pose.position.y - start_pose.position.y)
        waypoint.position.z = start_pose.position.z + alpha * (end_pose.position.z - start_pose.position.z)
        waypoint.orientation = start_pose.orientation
        waypoints.append(waypoint)

    return waypoints

def move_linear(move_group, waypoints):
    """
    各補間ポイントで逆運動学を使用してエンドエフェクタを直線的に移動させる

    :param move_group: MoveGroupCommander インスタンス
    :param waypoints: エンドエフェクタが通過するべきポイントのリスト
    """
    for waypoint in waypoints:
        move_group.set_pose_target(waypoint)
        plan = move_group.plan()
        if plan:
            move_group.execute(plan, wait=True)
        else:
            rospy.logwarn("逆運動学の計算が失敗しました")

def main():
    rospy.init_node('linear_move_example')

    # MoveIt!の初期化
    moveit_commander.roscpp_initialize(sys.argv)
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group_name = "ur5_arm"  # MoveIt!のグループ名に合わせて変更してください
    move_group = MoveGroupCommander(group_name)

    # エンドエフェクタの現在位置を取得
    start_pose = move_group.get_current_pose().pose

    # 目標位置を設定
    target_pose = Pose()
    target_pose.position.x = start_pose.position.x + 0.1  # 例: X方向に0.1m移動
    target_pose.position.y = start_pose.position.y
    target_pose.position.z = start_pose.position.z
    target_pose.orientation = start_pose.orientation

    # 直線経路を補間
    waypoints = interpolate_waypoints(start_pose, target_pose, step_size=0.01)

    # 各補間ポイントで逆運動学を使用してエンドエフェクタを直線的に移動
    move_linear(move_group, waypoints)

if __name__ == "__main__":
    main()
