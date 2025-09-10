#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <vector>

// 全局变量,用于存储关节名称和关节位置
std::vector<std::string> joint_names;
std::vector<double> joint_positions;

// 回调函数,用于处理/plan/joint_states话题
void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    joint_names = msg->name;
    joint_positions = msg->position;
}

// 回调函数,用于处理/wpr2_cmd话题
bool pub_joint_states = false;
void cmdCallback(const std_msgs::StringConstPtr& msg)
{
    if (msg->data == "go")
        pub_joint_states = true;
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "wpr2_arms_test");
    ros::NodeHandle nh;

    ROS_WARN("wpr2_arms_test start!");

    // 订阅/plan/joint_states话题
    ros::Subscriber joint_states_sub = nh.subscribe("/plan/joint_states", 10, jointStatesCallback);

    // 订阅/wpr2_cmd话题
    ros::Subscriber cmd_sub = nh.subscribe("/wpr2_cmd", 10, cmdCallback);

    // 不同关节组创建发布者
    ros::Publisher chest_height_pub = nh.advertise<sensor_msgs::JointState>("/real/wpr2/chest_height", 10);
    ros::Publisher left_arm_pub = nh.advertise<sensor_msgs::JointState>("/real/wpr2/left_arm", 10);
    ros::Publisher right_arm_pub = nh.advertise<sensor_msgs::JointState>("/real/wpr2/right_arm", 10);


    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // 如果接收到"go"命令,则发布/real/joint_states话题
        if (pub_joint_states)
        {
            // 输出接收到的关节名称和关节位置,进行一一对应显示
            ROS_INFO("接收到的虚拟机器人关节角度:");
            if (joint_names.size() == joint_positions.size() && !joint_names.empty())
            {
                for (size_t i = 0; i < joint_names.size(); ++i)
                {
                    ROS_INFO_STREAM(i << "  " << joint_names[i] << ": " << joint_positions[i]);
                }
            }
            else
            {
                ROS_ERROR("接收数据有误：Joint names 和 positions 的长度不一致或为空！");
                // 重置标记变量，避免重复处理错误数据
                pub_joint_states = false;
                ros::spinOnce();
                loop_rate.sleep();
                continue; // 跳过本次循环的后续部分
            }

            // ----发送关节控制指令开始---

            // 1. 发送胸部升降关节指令
            ROS_WARN("********** 正在发送胸部升降指令 **********");
            sensor_msgs::JointState chest_msg;
            chest_msg.header.stamp = ros::Time::now();
            if (joint_names.size() > 4 && joint_positions.size() > 4)
            {
                chest_msg.name.push_back(joint_names[4]);
                chest_msg.position.push_back(joint_positions[4]);
                chest_msg.velocity.push_back(18000);
                chest_height_pub.publish(chest_msg);
                ROS_INFO("发布到 /real/wpr2/chest_height: %s = %f", chest_msg.name[0].c_str(), chest_msg.position[0]);
            } else {
                ROS_ERROR("关节数据不足，无法发送胸部升降指令！");
            }
            ROS_WARN("****************************************");


            // 2. 发送左臂关节指令
            ROS_WARN("********** 正在发送左臂指令 **********");
            sensor_msgs::JointState left_arm_msg;
            left_arm_msg.header.stamp = ros::Time::now();
            if (joint_names.size() > 12 && joint_positions.size() > 12)
            {
                left_arm_msg.name.assign(joint_names.begin() + 7, joint_names.begin() + 13);
                left_arm_msg.position.assign(joint_positions.begin() + 7, joint_positions.begin() + 13);
                for(size_t i = 0; i < left_arm_msg.name.size(); ++i)
                {
                    left_arm_msg.velocity.push_back(2000);
                }
                left_arm_pub.publish(left_arm_msg);
                ROS_INFO("发布到 /real/wpr2/left_arm:");
                for(size_t i = 0; i < left_arm_msg.name.size(); ++i)
                {
                    ROS_INFO("  %s = %f", left_arm_msg.name[i].c_str(), left_arm_msg.position[i]);
                }
            } else {
                 ROS_ERROR("关节数据不足，无法发送左臂指令！");
            }
            ROS_WARN("**************************************");


            // 3. 发送右臂关节指令
            ROS_WARN("********** 正在发送右臂指令 **********");
            sensor_msgs::JointState right_arm_msg;
            right_arm_msg.header.stamp = ros::Time::now();
            if (joint_names.size() > 18 && joint_positions.size() > 18)
            {
                right_arm_msg.name.assign(joint_names.begin() + 13, joint_names.begin() + 19);
                right_arm_msg.position.assign(joint_positions.begin() + 13, joint_positions.begin() + 19);
                for(size_t i = 0; i < right_arm_msg.name.size(); ++i)
                {
                    right_arm_msg.velocity.push_back(2000);
                }
                right_arm_pub.publish(right_arm_msg);
                ROS_INFO("发布到 /real/wpr2/right_arm:");
                for(size_t i = 0; i < right_arm_msg.name.size(); ++i)
                {
                    ROS_INFO("  %s = %f", right_arm_msg.name[i].c_str(), right_arm_msg.position[i]);
                }
            } else {
                ROS_ERROR("关节数据不足，无法发送右臂指令！");
            }
            ROS_WARN("**************************************");

            // ----发送关节控制指令结束---

            // 重置标记变量
            pub_joint_states = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}