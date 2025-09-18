#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLineEdit, QGridLayout, QVBoxLayout, QLabel, QHBoxLayout, QSlider
from PyQt5.QtCore import Qt, QTimer
from geometry_msgs.msg import Twist
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

        # 速度话题发布者将在发送速度时动态创建
        self.cmd_vel_pub = None
        # 用于存储自动停止的定时器
        self.stop_timer = None

        # 初始化UI
        self.init_ui()

    def init_ui(self):
        # 窗口标题和大小
        self.setWindowTitle('WPR2 控制面板')
        self.setGeometry(300, 300, 450, 500) # 调整窗口大小以容纳新控件

        # === UI 控件 ===
        # 速度话题输入框
        self.topic_label = QLabel('速度话题:')
        self.topic_input = QLineEdit(self)
        self.topic_input.setText("cmd_vel")

        # 3x3 按钮网格
        grid_layout = QGridLayout()
        grid_layout.setSpacing(10)

        # 定义按钮及其位置、文本和回调函数
        buttons_info = {
            'turn_left':    (0, 0, '↶', self.turn_left),
            'forward':      (0, 1, '↑', self.move_forward),
            'turn_right':   (0, 2, '↷', self.turn_right),
            'left':         (1, 0, '←', self.move_left),
            'stop':         (1, 1, '■', self.stop_robot),
            'right':        (1, 2, '→', self.move_right),
            'backward':     (2, 1, '↓', self.move_backward),
        }

        # 创建并添加按钮到网格布局
        for name, (row, col, text, callback) in buttons_info.items():
            button = QPushButton(text, self)
            button.setFixedSize(80, 80)
            button.clicked.connect(callback)
            grid_layout.addWidget(button, row, col)

        # === 新增：手爪控制滑杆 ===
        # 左手爪滑杆
        left_gripper_layout = QHBoxLayout()
        left_gripper_slider = QSlider(Qt.Horizontal)
        # 修改: 将滑杆范围设置为 0-155，以便后续处理为 0-0.155
        left_gripper_slider.setRange(0, 155)
        self.left_gripper_value_label = QLabel('0.000') # 修改: 初始显示为浮点数
        left_gripper_layout.addWidget(QLabel('左手爪:'))
        left_gripper_layout.addWidget(left_gripper_slider)
        left_gripper_layout.addWidget(self.left_gripper_value_label)
        left_gripper_slider.valueChanged.connect(self.left_gripper_slider_changed)

        # 右手爪滑杆
        right_gripper_layout = QHBoxLayout()
        right_gripper_slider = QSlider(Qt.Horizontal)
        # 修改: 将滑杆范围设置为 0-155，以便后续处理为 0-0.155
        right_gripper_slider.setRange(0, 155)
        self.right_gripper_value_label = QLabel('0.000') # 修改: 初始显示为浮点数
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
        main_layout.addWidget(self.topic_label)
        main_layout.addWidget(self.topic_input)
        main_layout.addLayout(grid_layout)
        # 将滑杆布局添加到网格和底部按钮之间
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

    # --- 修改: 滑杆回调函数 ---
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


    def _get_velocity_publisher(self):
        """ 根据输入框中的话题名创建或复用发布者 """
        topic_name = self.topic_input.text()
        if not topic_name:
            rospy.logwarn("速度话题为空,无法发布.")
            return None
            
        if self.cmd_vel_pub is None or self.cmd_vel_pub.name != '/' + topic_name:
            rospy.loginfo(f"正在为话题 '{topic_name}' 创建发布者...")
            try:
                self.cmd_vel_pub = rospy.Publisher(topic_name, Twist, queue_size=10)
                rospy.sleep(0.5) # 等待发布者建立连接
            except Exception as e:
                rospy.logerr(f"创建发布者失败: {e}")
                return None
        return self.cmd_vel_pub

    def _publish_velocity(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, auto_stop=True):
        """ 发布速度指令，并按需设置一个3秒后停止的定时器 """
        pub = self._get_velocity_publisher()
        if pub is None:
            return

        if self.stop_timer is not None:
            self.stop_timer.shutdown()
            self.stop_timer = None
        
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        
        pub.publish(twist)
        rospy.loginfo(f"发布速度到话题 '{pub.name}': linear=({linear_x}, {linear_y}), angular={angular_z}")

        if auto_stop:
            self.stop_timer = rospy.Timer(rospy.Duration(3), self.stop_robot, oneshot=True)

    # --- 速度控制按钮回调函数 ---
    def move_forward(self):
        self._publish_velocity(linear_x=0.1)

    def move_backward(self):
        self._publish_velocity(linear_x=-0.1)

    def move_left(self):
        self._publish_velocity(linear_y=0.1)

    def move_right(self):
        self._publish_velocity(linear_y=-0.1)

    def turn_left(self):
        self._publish_velocity(angular_z=0.5)

    def turn_right(self):
        self._publish_velocity(angular_z=-0.5)

    def stop_robot(self, event=None):
        """ 停止机器人运动 """
        rospy.loginfo("发送停止指令")
        self._publish_velocity(auto_stop=False)

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