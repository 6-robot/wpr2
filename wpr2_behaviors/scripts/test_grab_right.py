#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import Float32
import sys

def publish_grab_value():
    """
    初始化ROS节点，并从命令行参数获取一个浮点数，
    然后将其发布到/wpr2/right_grab话题。
    """
    # 检查命令行参数是否足够
    if len(sys.argv) < 2:
        rospy.logerr("未提供数值参数！")
        rospy.logerr("用法: rosrun wpr2_behaviors test_grab_right.py [value]")
        sys.exit(1)

    # 尝试将参数转换为浮点数
    try:
        grab_value = float(sys.argv[1])
    except ValueError:
        rospy.logerr("无效的参数。请输入一个数字。")
        sys.exit(1)

    # 初始化发布者节点
    # anonymous=True 确保了每个节点都有一个唯一的名字
    rospy.init_node('test_grab_right_publisher', anonymous=True)

    # 创建一个Publisher，发布到/wpr2/right_grab话题，消息类型为Float32
    pub = rospy.Publisher('/wpr2/right_grab', Float32, queue_size=10)

    # 等待，以确保发布者与roscore建立连接
    rospy.sleep(1.0)

    # 创建Float32消息
    msg_to_publish = Float32()
    msg_to_publish.data = grab_value

    # 发布消息
    rospy.loginfo("正在向 /wpr2/right_grab 话题发布数值: %f", grab_value)
    pub.publish(msg_to_publish)

    # 短暂延时，确保消息已发出
    rospy.sleep(0.5)

    rospy.loginfo("消息已发布，节点即将关闭。")


if __name__ == '__main__':
    try:
        publish_grab_value()
    except rospy.ROSInterruptException:
        # 捕获因Ctrl+C等操作引起的ROS中断异常
        pass