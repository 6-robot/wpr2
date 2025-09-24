#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

class TrajectoryActionBridge(object):
    def __init__(self, arm_name, topic_name, lift):
        self._arm_name = arm_name
        self._topic_name = topic_name
        self._lift = lift

        # 创建一个Publisher来发送升降
        self._lift_pub = rospy.Publisher("/wpr2/chest_height", JointState, queue_size=10)
        
        # 创建一个Publisher来发送JointState消息
        self._pub = rospy.Publisher(self._topic_name, JointState, queue_size=10)
        
        # Action服务器的名称需要和controllers.yaml中定义的完全一致
        if( self._lift == False):
            action_server_name = self._arm_name + '_controller/follow_joint_trajectory'
        else:
            action_server_name = self._arm_name + '_lift_controller/follow_joint_trajectory'
        self._as = actionlib.SimpleActionServer(
            action_server_name,
            FollowJointTrajectoryAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()
        rospy.loginfo("Action server for " + self._arm_name + " started.")

    def execute_cb(self, goal):
        rospy.loginfo("收到运动轨迹 " + self._arm_name)
        
        # 获取关节名称和轨迹点
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points

        # 打印关节名称
        rospy.loginfo("关节名称列表: " + str(joint_names))


        # 检查是否有轨迹点
        if not trajectory_points:
            rospy.logwarn("接收到的规划轨迹为空...")
            self._as.set_succeeded(FollowJointTrajectoryResult())
            return

        # 记录轨迹开始时间
        start_time = rospy.Time.now()
        
        # 循环遍历轨迹中的每个点
        for point in trajectory_points:
            # 计算当前点应该在何时执行
            time_to_reach = start_time + point.time_from_start
            
            # 等待直到达到预定时间
            while rospy.Time.now() < time_to_reach:
                # 允许小的提前量以应对延迟
                if rospy.is_shutdown():
                    return
                rospy.sleep(0.01)

            # 创建并填充JointState消息
            if( self._lift == False):
                js_msg = JointState()
                js_msg.header.stamp = rospy.Time.now()
                js_msg.name = joint_names
                js_msg.position = point.positions
                js_msg.velocity = [2000] * len(joint_names)
                js_msg.effort = []
                self._pub.publish(js_msg)
                # rospy.loginfo("发送关节指令: " + str(js_msg.name))
            else:
                # --- 1. 处理升降关节 (第一个成员) ---
                lift_msg = JointState()
                lift_msg.header.stamp = rospy.Time.now()
                
                # 使用 [list[0]] 的方式获取第一个成员，并保持其为列表格式
                lift_msg.name = [joint_names[0]]
                lift_msg.position = [point.positions[0]]
                lift_msg.velocity = [10000]
                lift_msg.effort = []
                
                # 发布升降指令
                self._lift_pub.publish(lift_msg)
                # rospy.loginfo("发送升降指令: " + str(lift_msg.name))

                # --- 2. 处理手臂关节 (剩下的成员) ---
                js_msg = JointState()
                js_msg.header.stamp = rospy.Time.now()

                # 使用 [1:] 切片获取从第二个到最后的所有成员
                js_msg.name = joint_names[1:]
                js_msg.position = point.positions[1:]
                js_msg.velocity = [2000] * len(js_msg.name)
                js_msg.effort = []
                
                # 发布手臂关节指令
                self._pub.publish(js_msg)
                # rospy.loginfo("发送关节指令: " + str(js_msg.name))

        rospy.loginfo( self._arm_name + " 的规划路径执行完毕！")
        # 设置Action结果为成功
        self._as.set_succeeded(FollowJointTrajectoryResult())

if __name__ == '__main__':
    rospy.init_node('trajectory_action_bridge')
    
    # 为左臂和右臂分别创建一个桥梁实例
    left_arm_bridge = TrajectoryActionBridge('left_arm', '/wpr2/left_arm', False)
    right_arm_bridge = TrajectoryActionBridge('right_arm', '/wpr2/right_arm', False)
    left_lift_bridge = TrajectoryActionBridge('left_arm', '/wpr2/left_arm', True)
    right_lift_bridge = TrajectoryActionBridge('right_arm', '/wpr2/right_arm', True)
    
    rospy.spin()