#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tf
import numpy as np

def set_center_and_calculate_new_pose():
    """
    新しい回転中心を設定し、その中心を基にTCPを移動させた新しい位置を計算します。
    """
    # 現在のエンドエフェクタの位置を取得
    current_pose_stamped = group.get_current_pose()
    current_pose = current_pose_stamped.pose

    # 回転中心の設定（エンドエフェクタに対するオフセット）
    center_offset = geometry_msgs.msg.Vector3()
    center_offset.x = -0.04 # 回転中心のXオフセット
    center_offset.y = 0.04  # 回転中心のYオフセット
    center_offset.z = 0.03  # 回転中心のZオフセット

    # 回転中心のワールド座標系での位置を計算
    center_world = geometry_msgs.msg.Point()
    center_world.x = current_pose.position.x + center_offset.x
    center_world.y = current_pose.position.y + center_offset.y
    center_world.z = current_pose.position.z + center_offset.z

    return current_pose, center_world

def calculate_new_pose(current_pose, center_world, angle, axis, steps):
    """
    指定された軸と角度に基づき、エンドエフェクタの新しい位置とオリエンテーションを計算します。
    """
    poses = []

    for i in range(1, steps + 1):
        step_angle = (angle / steps) * i
        rotation_matrix = tf.rotation_matrix(step_angle, axis)

        # エンドエフェクタの現在位置をベクトルとして取得し、回転中心に相対的に移動
        current_position = np.array([current_pose.position.x - center_world.x,
                                     current_pose.position.y - center_world.y,
                                     current_pose.position.z - center_world.z, 1])

        # 新しい位置を計算
        rotated_position = np.dot(rotation_matrix, current_position)
        new_position = rotated_position[:3]

        # 回転中心に向かう方向を計算
        direction = -new_position
        direction /= np.linalg.norm(direction)

        # 新しいオリエンテーションを計算（回転中心を向くように）
        z_axis = np.array([0, 0, 1])
        rotation_axis = np.cross(z_axis, direction)
        rotation_angle = np.arccos(np.dot(z_axis, direction))
        new_orientation_quat = tf.quaternion_about_axis(rotation_angle, rotation_axis)

        # 新しいエンドエフェクタの位置とオリエンテーション
        new_pose = geometry_msgs.msg.Pose()
        new_pose.position.x = new_position[0] + center_world.x
        new_pose.position.y = new_position[1] + center_world.y
        new_pose.position.z = new_position[2] + center_world.z
        new_pose.orientation.x = new_orientation_quat[0]
        new_pose.orientation.y = new_orientation_quat[1]
        new_pose.orientation.z = new_orientation_quat[2]
        new_pose.orientation.w = new_orientation_quat[3]

        poses.append(new_pose)

    return poses

def calculate_new_poses_for_roll(current_pose, center_world, steps=10):
    """
    ロール方向に90度回転させた新しいTCPの位置を複数ステップで計算します。
    """
    angle = np.pi / 9  # 90度をラジアンに変換
    axis = [1, 0, 0]  # ロール軸
    return calculate_new_pose(current_pose, center_world, angle, axis, steps)

def calculate_new_poses_for_pitch(current_pose, center_world, steps=10):
    """
    ピッチ方向に90度回転させた新しいTCPの位置を複数ステップで計算します.
    """
    angle = np.pi / 9  # 90度をラジアンに変換
    axis = [0, 1, 0]  # ピッチ軸
    return calculate_new_pose(current_pose, center_world, angle, axis, steps)

def calculate_new_poses_for_yaw(current_pose, center_world, steps=10):
    """
    ヨー方向に90度回転させた新しいTCPの位置を複数ステップで計算します.
    """
    angle = np.pi / 9  # 90度をラジアンに変換
    axis = [0, 0, 1]  # ヨー軸
    return calculate_new_pose(current_pose, center_world, angle, axis, steps)

def execute_movements(group, poses):
    """
    計算された新しいTCPの位置をMoveItを使用して順次実行します.
    """
    for pose in poses:
        group.set_pose_target(pose)
        plan = group.plan()
        if plan:
            trajectory = plan[1]  # plan[1] に trajectory が含まれる
            group.execute(trajectory, wait=True)
        else:
            rospy.logerr("Planning failed")

if __name__ == "__main__":
    # ROSノードの初期化
    rospy.init_node('set_center_and_move', anonymous=True)

    # MoveItの初期化
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("ur5_arm")

    # 新たな回転中心を設定し、現在の姿勢を取得
    current_pose, center_world = set_center_and_calculate_new_pose()

    # 各軸に対する回転動作を複数ステップで実行
    for move_func in [calculate_new_poses_for_roll, calculate_new_poses_for_pitch, calculate_new_poses_for_yaw]:
        # 新しいTCPの位置を計算
        poses = move_func(current_pose, center_world)
        
        # 計算された新しい位置で回転を実行
        execute_movements(group, poses)
        
        # 新しい姿勢を次の回転の現在姿勢として設定
        current_pose = poses[-1]

    # ROSのシャットダウン
    moveit_commander.roscpp_shutdown()
