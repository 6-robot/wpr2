#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

ros::Publisher chest_height_pub;
ros::Publisher left_arm_pub;
ros::Publisher right_arm_pub;
ros::Publisher left_gripper_pub;
ros::Publisher right_gripper_pub;

sensor_msgs::JointState chest_height_msg;
sensor_msgs::JointState left_arm_msg;
sensor_msgs::JointState right_arm_msg;
sensor_msgs::JointState left_gripper_msg;
sensor_msgs::JointState right_gripper_msg;

double chest_position;
double left_position[6];
double right_position[6];

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpr2_joints_demo");

    ros::NodeHandle n;
    chest_height_pub = n.advertise<sensor_msgs::JointState>("/wpr2/chest_height", 10);
    left_arm_pub = n.advertise<sensor_msgs::JointState>("/wpr2/left_arm", 10);
    left_gripper_pub = n.advertise<sensor_msgs::JointState>("/wpr2/left_gripper", 10);
    right_arm_pub = n.advertise<sensor_msgs::JointState>("/wpr2/right_arm", 10);
    right_gripper_pub = n.advertise<sensor_msgs::JointState>("/wpr2/right_gripper", 10);

    // 升降
    chest_height_msg.name.resize(1);
    chest_height_msg.position.resize(1);
    chest_height_msg.velocity.resize(1);
    chest_height_msg.name[0] = "chest_height";
    chest_height_msg.position[0] = 0.7;
    chest_height_msg.velocity[0] = 15000;

    // 左臂
    left_arm_msg.name.resize(6);
    left_arm_msg.position.resize(6);
    left_arm_msg.velocity.resize(6);
    for(int i=0;i<6;i++)
    {
        left_arm_msg.name[i] = "left_name_" + std::to_string(i+1);
        left_arm_msg.position[i] = left_position[i];
        left_arm_msg.velocity[i] = 2000;
    }

    // 左手爪
    left_gripper_msg.name.resize(1);
    left_gripper_msg.position.resize(1);
    left_gripper_msg.velocity.resize(1);
    left_gripper_msg.name[0] = "left_gripper";
    left_gripper_msg.position[0] = 0.1;
    left_gripper_msg.velocity[0] = 2000;

    // 右臂
    right_arm_msg.name.resize(6);
    right_arm_msg.position.resize(6);
    right_arm_msg.velocity.resize(6);
    for(int i=0;i<6;i++)
    {
        right_arm_msg.name[i] = "right_name_" + std::to_string(i+1);
        right_arm_msg.position[i] = right_position[i];
        right_arm_msg.velocity[i] = 2000;
    }

    // 右手爪
    right_gripper_msg.name.resize(1);
    right_gripper_msg.position.resize(1);
    right_gripper_msg.velocity.resize(1);
    right_gripper_msg.name[0] = "right_gripper";
    right_gripper_msg.position[0] = 0.1;
    right_gripper_msg.velocity[0] = 2000;

    int nCount = 0;
    ros::Rate r(0.2);
    ros::spinOnce();
    r.sleep();
    
    while(ros::ok())
    {
        ROS_WARN("[wpr2_joints_demo] nCount = %d", nCount);
        switch(nCount)
        {
        case 0:
            // 第一个姿势 
            chest_height_msg.position[0] = 0.65;
            chest_height_msg.velocity[0] = 15000;
            chest_height_pub.publish(chest_height_msg);

            left_position[0] = 0.5;
            left_position[1] = -1.57;
            left_position[2] = 0;
            left_position[3] = -1.16;
            left_position[4] = 0;
            left_position[5] = 0;
            for(int i=0;i<6;i++)
            {
                left_arm_msg.position[i] = left_position[i];
                left_arm_msg.velocity[i] = 2000;
            }
            left_arm_pub.publish(left_arm_msg);

            left_gripper_msg.position[0] = 0.05;
            left_gripper_msg.velocity[0] = 2000;
            left_gripper_pub.publish(left_gripper_msg);

            right_position[0] = -0.5;
            right_position[1] = 1.57;
            right_position[2] = 0.0;
            right_position[3] = 1.16;
            right_position[4] = 0;
            right_position[5] = 0;
            for(int i=0;i<6;i++)
            {
                right_arm_msg.position[i] = right_position[i];
                right_arm_msg.velocity[i] = 2000;
            }
            right_arm_pub.publish(right_arm_msg);

            right_gripper_msg.position[0] = 0.05;
            right_gripper_msg.velocity[0] = 2000;
            right_gripper_pub.publish(right_gripper_msg);

            break;
        case 1:
            // 第二个姿势
            chest_height_msg.position[0] = 0.7;
            chest_height_msg.velocity[0] = 15000;
            chest_height_pub.publish(chest_height_msg);

            left_position[0] = 0.3;
            left_position[1] = -1.4;
            left_position[2] = 0.1;
            left_position[3] = -0.6;
            left_position[4] = 0.4;
            left_position[5] = -0.2;
            for(int i=0;i<6;i++)
            {
                left_arm_msg.position[i] = left_position[i];
                left_arm_msg.velocity[i] = 2000;
            }
            left_arm_pub.publish(left_arm_msg);

            left_gripper_msg.position[0] = 0.1;
            left_gripper_msg.velocity[0] = 2000;
            left_gripper_pub.publish(left_gripper_msg);

            right_position[0] = -0.3;
            right_position[1] = 1.4;
            right_position[2] = -0.1;
            right_position[3] = 0.6;
            right_position[4] = -0.4;
            right_position[5] = 0.2;
            for(int i=0;i<6;i++)
            {
                right_arm_msg.position[i] = right_position[i];
                right_arm_msg.velocity[i] = 2000;
            }
            right_arm_pub.publish(right_arm_msg);

            right_gripper_msg.position[0] = 0.1;
            right_gripper_msg.velocity[0] = 2000;
            right_gripper_pub.publish(right_gripper_msg);
            break;
        
        }
        ros::spinOnce();
        r.sleep();
        nCount ++;
        if(nCount > 1)
        {
            nCount = 0;
        }
    }

    return 0;
}