#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLineEdit, QGridLayout, QVBoxLayout, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer 
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class WPR2Panel(QWidget):
    def __init__(self):
        super().__init__()
        # 为 wpr2_cmd 话题创建发布者
        self.wpr2_cmd_pub = rospy.Publisher('wpr2_cmd', String, queue_size=10)
        
        # 速度话题发布者将在发送速度时动态创建
        self.cmd_vel_pub = None
        # 用于存储自动停止的定时器
        self.stop_timer = None

        # 初始化UI
        self.init_ui()

    def init_ui(self):
        # 窗口标题和大小
        self.setWindowTitle('WPR2 控制面板')
        self.setGeometry(300, 300, 350, 400)

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
        main_layout.addLayout(bottom_layout)
        
        self.setLayout(main_layout)

        # 2. 创建并启动一个定时器来检查ROS关闭信号
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.check_ros_shutdown)
        self.ros_timer.start(100)  # 设置检查间隔为100毫秒

        self.show()

    # 3. 检查ROS状态并关闭UI的方法
    def check_ros_shutdown(self):
        """
        定期检查 rospy.is_shutdown()。如果为True，则关闭Qt应用。
        """
        if rospy.is_shutdown():
            # 当ROS关闭时，这个方法会关闭PyQt窗口，
            # 从而让 app.exec_() 返回，程序得以干净地退出。
            self.close()

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

        # 如果存在旧的定时器，先取消它
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
            # 创建一个一次性的定时器，3秒后调用 stop_robot
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
    # 首先初始化 ROS 节点
    rospy.init_node('wpr2_panel', anonymous=True)
    
    # 然后创建Qt应用和窗口
    app = QApplication(sys.argv)
    panel = WPR2Panel()
    
    # 运行Qt事件循环。当窗口关闭时，app.exec_()会返回。
    # sys.exit() 确保程序以正确的状态码退出。
    # 之前的 try/except 块在这里不再需要，因为关闭逻辑已由QTimer处理。
    sys.exit(app.exec_())