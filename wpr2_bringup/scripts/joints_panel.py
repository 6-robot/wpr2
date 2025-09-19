#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout, QVBoxLayout, QLabel, QHBoxLayout, QSlider
from PyQt5.QtCore import Qt, QTimer
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class WPR2Panel(QWidget):
    def __init__(self):
        super().__init__()
        # 为 wpr2_cmd 话题创建发布者
        self.wpr2_cmd_pub = rospy.Publisher('wpr2_cmd', String, queue_size=10)
        
        # 为左右手爪创建发布者
        self.left_gripper_pub = rospy.Publisher('/real/wpr2/left_gripper', JointState, queue_size=10)
        self.right_gripper_pub = rospy.Publisher('/real/wpr2/right_gripper', JointState, queue_size=10)

        # 初始化UI
        self.init_ui()

    def init_ui(self):
        # 窗口标题和大小
        self.setWindowTitle('WPR2 控制面板')
        self.setGeometry(300, 300, 450, 250) # 调整窗口大小以适应减少的控件

        # === UI 控件 ===

        # === 手爪控制滑杆 ===
        # 左手爪滑杆
        left_gripper_layout = QHBoxLayout()
        left_gripper_slider = QSlider(Qt.Horizontal)
        # 将滑杆范围设置为 0-155，以便后续处理为 0-0.155
        left_gripper_slider.setRange(0, 155)
        self.left_gripper_value_label = QLabel('0.000') # 初始显示为浮点数
        left_gripper_layout.addWidget(QLabel('左手爪:'))
        left_gripper_layout.addWidget(left_gripper_slider)
        left_gripper_layout.addWidget(self.left_gripper_value_label)
        left_gripper_slider.valueChanged.connect(self.left_gripper_slider_changed)

        # 右手爪滑杆
        right_gripper_layout = QHBoxLayout()
        right_gripper_slider = QSlider(Qt.Horizontal)
        # 将滑杆范围设置为 0-155，以便后续处理为 0-0.155
        right_gripper_slider.setRange(0, 155)
        self.right_gripper_value_label = QLabel('0.000') # 初始显示为浮点数
        right_gripper_layout.addWidget(QLabel('右手爪:'))
        right_gripper_layout.addWidget(right_gripper_slider)
        right_gripper_layout.addWidget(self.right_gripper_value_label)
        right_gripper_slider.valueChanged.connect(self.right_gripper_slider_changed)


        # 下方的 "reset" 和 "go" 按钮
        bottom_layout = QHBoxLayout()
        reset_button = QPushButton('重置 (reset)', self)
        reset_button.clicked.connect(self.send_reset_cmd)
        go_button = QPushButton('执行 (go)', self)
        go_button.clicked.connect(self.send_go_cmd)
        bottom_layout.addWidget(reset_button)
        bottom_layout.addWidget(go_button)

        # === 整体布局 ===
        main_layout = QVBoxLayout()
        # 将滑杆布局和底部按钮添加到主布局
        main_layout.addLayout(left_gripper_layout)
        main_layout.addLayout(right_gripper_layout)
        main_layout.addLayout(bottom_layout)
        
        self.setLayout(main_layout)

        # 创建并启动一个定时器来检查ROS关闭信号
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.check_ros_shutdown)
        self.ros_timer.start(100)  # 设置检查间隔为100毫秒

        self.show()

    def check_ros_shutdown(self):
        """
        定期检查 rospy.is_shutdown()。如果为True，则关闭Qt应用。
        """
        if rospy.is_shutdown():
            self.close()

    # --- 滑杆回调函数 ---
    def left_gripper_slider_changed(self, value):
        """ 左手爪滑杆值变化时的回调函数 """
        # 将滑杆的整数值 (0-155) 转换为浮点数值 (0-0.155)
        position_value = value / 1000.0
        
        # 更新UI上的数值显示，格式化为三位小数
        self.left_gripper_value_label.setText(f"{position_value:.3f}")
        
        # 创建并填充JointState消息
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.position = [position_value] # 使用转换后的浮点数
        joint_state_msg.velocity = [2000]
        
        # 发布消息
        self.left_gripper_pub.publish(joint_state_msg)
        # rospy.loginfo(f"发布到 'wpr2/left_gripper': position={position_value}, velocity=2000")

    def right_gripper_slider_changed(self, value):
        """ 右手爪滑杆值变化时的回调函数 """
        # 将滑杆的整数值 (0-155) 转换为浮点数值 (0-0.155)
        position_value = value / 1000.0

        # 更新UI上的数值显示，格式化为三位小数
        self.right_gripper_value_label.setText(f"{position_value:.3f}")
        
        # 创建并填充JointState消息
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.position = [position_value] # 使用转换后的浮点数
        joint_state_msg.velocity = [2000]
        
        # 发布消息
        self.right_gripper_pub.publish(joint_state_msg)
        # rospy.loginfo(f"发布到 'wpr2/right_gripper': position={position_value}, velocity=2000")

    # --- wpr2_cmd 按钮回调函数 ---
    def send_reset_cmd(self):
        """ 向 wpr2_cmd 话题发送 'reset' 字符串 """
        cmd_str = String()
        cmd_str.data = "reset"
        self.wpr2_cmd_pub.publish(cmd_str)
        rospy.loginfo("向 'wpr2_cmd' 话题发送: reset")

    def send_go_cmd(self):
        """ 向 wpr2_cmd 话题发送 'go' 字符串 """
        cmd_str = String()
        cmd_str.data = "go"
        self.wpr2_cmd_pub.publish(cmd_str)
        rospy.loginfo("向 'wpr2_cmd' 话题发送: go")


if __name__ == '__main__':
    rospy.init_node('wpr2_panel', anonymous=True)
    
    app = QApplication(sys.argv)
    panel = WPR2Panel()
    
    sys.exit(app.exec_())