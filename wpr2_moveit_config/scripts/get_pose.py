#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import tf
from sensor_msgs.msg import JointState
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QGroupBox, QTextEdit, QHBoxLayout, QPushButton, QMessageBox)
from PyQt5.QtCore import Qt, QTimer

class PoseDisplayGUI(QWidget):
    def __init__(self):
        super(PoseDisplayGUI, self).__init__()

        # 初始化ROS节点
        try:
            rospy.init_node('pose_display_gui', anonymous=True)
        except rospy.ROSInitException as e:
            print("无法连接到ROS Master，请确保roscore正在运行。")
            sys.exit(1)


        # 初始化TF监听器
        self.tf_listener = tf.TransformListener()

        # 初始化UI
        self.init_ui()
        
        # 创建一个定时器来检查ROS master的状态
        self.ros_check_timer = QTimer(self)
        # 每1000毫秒（1秒）检查一次
        self.ros_check_timer.timeout.connect(self.check_ros_master)
        self.ros_check_timer.start(1000)

    def init_ui(self):
        # 主垂直布局
        main_layout = QVBoxLayout()

        # 末端目标姿态 GroupBox
        pose_groupbox = QGroupBox("末端目标姿态")
        pose_layout = QVBoxLayout()
        self.pose_editor = QTextEdit()
        self.pose_editor.setFixedSize(500, 200)
        self.pose_editor.setReadOnly(True)
        pose_layout.addWidget(self.pose_editor)
        pose_groupbox.setLayout(pose_layout)
        main_layout.addWidget(pose_groupbox)

        # 当前关节姿态 GroupBox
        joint_groupbox = QGroupBox("当前关节姿态")
        joint_layout = QVBoxLayout()
        self.joint_editor = QTextEdit()
        self.joint_editor.setFixedSize(500, 400)
        self.joint_editor.setReadOnly(True)
        joint_layout.addWidget(self.joint_editor)
        joint_groupbox.setLayout(joint_layout)
        main_layout.addWidget(joint_groupbox)

        # 按钮水平布局
        button_layout = QHBoxLayout()
        button_layout.setAlignment(Qt.AlignRight)

        # 左臂姿态按钮
        left_arm_button = QPushButton("左臂姿态")
        left_arm_button.clicked.connect(self.get_left_arm_pose)
        button_layout.addWidget(left_arm_button)

        # 右臂姿态按钮
        right_arm_button = QPushButton("右臂姿态")
        right_arm_button.clicked.connect(self.get_right_arm_pose)
        button_layout.addWidget(right_arm_button)

        main_layout.addLayout(button_layout)

        self.setLayout(main_layout)
        self.setWindowTitle('机器人姿态获取工具')
        self.show()
        
    # --- 新增函数 ---
    def check_ros_master(self):
        """
        定时检查ROS master是否仍在运行，如果不在则退出应用
        """
        if rospy.is_shutdown():
            print("ROS Master已关闭，正在退出GUI...")
            QApplication.instance().quit()


    def get_arm_pose(self, arm_prefix):
        """
        通用的获取手臂姿态和关节角度的函数
        """
        if rospy.is_shutdown():
            QMessageBox.warning(self, "警告", "ROS Master已关闭，无法执行操作。")
            return
            
        try:
            # 1. 从TF获取末端姿态
            target_link = arm_prefix + "_tcp_link"
            source_frame = "base_footprint"
            self.tf_listener.waitForTransform(source_frame, target_link, rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform(source_frame, target_link, rospy.Time(0))

            pose_code = (
                "geometry_msgs::Pose target_pose;\n"
                "target_pose.position.x = {:.4f};\n"
                "target_pose.position.y = {:.4f};\n"
                "target_pose.position.z = {:.4f};\n"
                "target_pose.orientation.x = {:.4f};\n"
                "target_pose.orientation.y = {:.4f};\n"
                "target_pose.orientation.z = {:.4f};\n"
                "target_pose.orientation.w = {:.4f};"
            ).format(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3])

            self.pose_editor.setText(pose_code)

            # 2. 从/joint_states获取关节角度
            joint_states = rospy.wait_for_message("/joint_states", JointState, timeout=5)
            
            joint_names_to_find = [arm_prefix + "_joint_" + str(i) for i in range(1, 7)]
            joint_names_to_find.append("chest_height")
            
            joint_positions = {}
            for name in joint_names_to_find:
                try:
                    index = joint_states.name.index(name)
                    joint_positions[name] = joint_states.position[index]
                except ValueError:
                    QMessageBox.warning(self, "警告", "在/joint_states中未找到关节: {}".format(name))
                    return
            
            chest_height = joint_positions.get("chest_height", 0.0)
            
            joint_code = "chest_height = {:.4f};\n\n".format(chest_height)
            
            for i in range(1, 7):
                joint_name = "{}_joint_{}".format(arm_prefix, i)
                joint_code += "{}_name[{}] = \"{}\";\n".format(arm_prefix, i-1, joint_name)
            
            joint_code += "\n"
            
            for i in range(1, 7):
                joint_name = "{}_joint_{}".format(arm_prefix, i)
                joint_code += "{}_position[{}] = {:.4f};\n".format(arm_prefix, i-1, joint_positions[joint_name])

            self.joint_editor.setText(joint_code)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            QMessageBox.critical(self, "TF错误", "获取TF变换失败: {}".format(e))
        except rospy.ROSException as e:
            QMessageBox.critical(self, "ROS错误", "从/joint_states获取消息失败: {}".format(e))
        except Exception as e:
            QMessageBox.critical(self, "未知错误", str(e))

    def get_left_arm_pose(self):
        self.get_arm_pose("left")

    def get_right_arm_pose(self):
        self.get_arm_pose("right")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = PoseDisplayGUI()
    sys.exit(app.exec_())