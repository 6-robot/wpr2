#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
A test node to publish a single Pose message to the /wpr2/grab_bottle topic.
"""

import rospy
from geometry_msgs.msg import Pose
import time

def publish_grab_bottle_pose():
    """
    Initializes the ROS node, publishes a Pose message, and then exits.
    """
    # 初始化发布者，指定话题名称、消息类型和队列大小
    pub = rospy.Publisher('/wpr2/grab_bottle', Pose, queue_size=10)

    # 初始化ROS节点，anonymous=True确保节点名称的唯一性
    rospy.init_node('test_bottle_publisher', anonymous=True)

    # 等待一小段时间，以确保发布者和订阅者之间建立了连接
    rospy.sleep(1.0)

    # 创建一个新的Pose消息实例
    grab_pose = Pose()

    # 填充消息的position字段
    grab_pose.position.x = 1.0
    grab_pose.position.y = 0.0
    grab_pose.position.z = 0.83

    # 填充消息的orientation字段 (使用四元数表示)
    # 此处使用一个单位四元数，表示没有旋转
    grab_pose.orientation.x = 0.0
    grab_pose.orientation.y = 0.0
    grab_pose.orientation.z = 0.0
    grab_pose.orientation.w = 1.0

    # 记录将要发布的消息内容
    rospy.loginfo("Publishing pose to /wpr2/grab_bottle:")
    rospy.loginfo(grab_pose)

    # 发布消息
    pub.publish(grab_pose)

    # 记录将要暂停的信息
    rospy.loginfo("Message published. Pausing for 1 second before exiting.")

    # 暂停一秒钟
    time.sleep(1)

    # 记录节点退出的信息
    rospy.loginfo("Exiting node.")

if __name__ == '__main__':
    try:
        publish_grab_bottle_pose()
    except rospy.ROSInterruptException:
        pass