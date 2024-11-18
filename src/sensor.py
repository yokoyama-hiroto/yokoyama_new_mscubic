#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import time
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose
from geometry_msgs.msg import WrenchStamped
import threading

# グローバル変数として力センサの値を保持
force_sensor_value = 0.0
sleep_time = 0.1 #sleeping time
force_sensor_lock = threading.Lock()

def force_sensor_callback(msg):
    global force_sensor_value
    with force_sensor_lock:
        force_sensor_value = msg.wrench.force.z
    print(force_sensor_value)

def get_current_tool_pose(group_arm):
    # ツールリンク（Tool Link）"tool0"の現在の姿勢を取得
    current_pose = group_arm.get_current_pose("ee_link").pose
    return current_pose

def move_relative():
    global force_sensor_value
    global group_arm
    # 閾値を設定（この値より小さい場合に動作を停止）
    threshold_value = 40.0
    
    # z方向に動く量
    relative_pose = Pose()
    relative_pose.position.z = 0.001  # z軸方向に0.01m移動

    group_arm.set_max_velocity_scaling_factor(0.1)

    # 力センサの値が閾値より小さい間、繰り返し移動を行う
    while not rospy.is_shutdown():
        with force_sensor_lock:
            if abs(force_sensor_value) < threshold_value:
                # 現在の姿勢を取得
                current_pose = get_current_tool_pose(group_arm)

                # 相対的な移動量を計算
                target_pose = Pose()
                target_pose.orientation = current_pose.orientation
                target_pose.position.x = current_pose.position.x
                target_pose.position.y = current_pose.position.y
                target_pose.position.z = current_pose.position.z + relative_pose.position.z

                # 目標姿勢を設定して移動
                group_arm.set_pose_target(target_pose)
                group_arm.go()
                time.sleep(sleep_time)
                print("a")
            else:
                # 現在の姿勢を取得
                current_pose = get_current_tool_pose(group_arm)

                # 相対的な移動量を計算
                target_pose = Pose()
                target_pose.orientation = current_pose.orientation
                target_pose.position.x = current_pose.position.x
                target_pose.position.y = current_pose.position.y
                target_pose.position.z = current_pose.position.z - relative_pose.position.z

                # 目標姿勢を設定して移動
                group_arm.set_pose_target(target_pose)
                group_arm.go()
                time.sleep(sleep_time)
                print("b")
                

if __name__ == "__main__":
    rospy.init_node('move_group_interface_tutorial')

    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander("ur5_arm")


    # 力センサのトピックを購読し、コールバック関数を設定
    rospy.Subscriber('ft_sensor/raw', WrenchStamped, force_sensor_callback)

    # ロボットの制御を行うスレッドを開始
    control_thread = threading.Thread(target=move_relative())
    control_thread.start()

    # センサの値の読み取りを行うスレッドを開始
    #sensor_thread = threading.Thread(target=rospy.spin())
    #sensor_thread.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Ctrl+Cが検出されました。プログラムを終了します。")

    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown(0)
