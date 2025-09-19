/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2025-2035, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* @author Zhang Wanjie                                             */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <string>

ros::Publisher nav_pub;
ros::Publisher chest_height_pub;
ros::Publisher left_arm_pub;
ros::Publisher right_arm_pub;

std::string a_left_name[6];
double a_left_position[6];
std::string a_right_name[6];
double a_right_position[6];

std::string b_left_name[6];
double b_left_position[6];
std::string b_right_name[6];
double b_right_position[6];

int nState = 0;

void init()
{
    // 第一组动作
    a_left_name[0] = "left_joint_1";
    a_left_name[1] = "left_joint_2";
    a_left_name[2] = "left_joint_3";
    a_left_name[3] = "left_joint_4";
    a_left_name[4] = "left_joint_5";
    a_left_name[5] = "left_joint_6";

    a_left_position[0] = 0.5;
    a_left_position[1] = -1.57;
    a_left_position[2] = 0;
    a_left_position[3] = -1.16;
    a_left_position[4] = 0;
    a_left_position[5] = 0;

    a_right_name[0] = "right_joint_1";
    a_right_name[1] = "right_joint_2";
    a_right_name[2] = "right_joint_3";
    a_right_name[3] = "right_joint_4";
    a_right_name[4] = "right_joint_5";
    a_right_name[5] = "right_joint_6";

    a_right_position[0] = -0.5;
    a_right_position[1] = 1.57;
    a_right_position[2] = 0.0;
    a_right_position[3] = 1.16;
    a_right_position[4] = 0;
    a_right_position[5] = 0;

    // 第二组动作
    b_left_name[0] = "left_joint_1";
    b_left_name[1] = "left_joint_2";
    b_left_name[2] = "left_joint_3";
    b_left_name[3] = "left_joint_4";
    b_left_name[4] = "left_joint_5";
    b_left_name[5] = "left_joint_6";

    b_left_position[0] = 0.3;
    b_left_position[1] = -1.4;
    b_left_position[2] = 0.1;
    b_left_position[3] = -0.6;
    b_left_position[4] = 0.4;
    b_left_position[5] = -0.2;

    b_right_name[0] = "right_joint_1";
    b_right_name[1] = "right_joint_2";
    b_right_name[2] = "right_joint_3";
    b_right_name[3] = "right_joint_4";
    b_right_name[4] = "right_joint_5";
    b_right_name[5] = "right_joint_6";

    b_right_position[0] = -0.3;
    b_right_position[1] = 1.4;
    b_right_position[2] = -0.1;
    b_right_position[3] = 0.6;
    b_right_position[4] = -0.4;
    b_right_position[5] = 0.2;

}

static int count = 0;

void NavResultCallback(const std_msgs::String &msg)
{
    ROS_WARN("[NavResultCallback] %s",msg.data.c_str());
    if(nState == 1)
    {
        ROS_WARN("达到 1 航点，做一个动作");
        sensor_msgs::JointState chest_msg;
        chest_msg.header.stamp = ros::Time::now();
        chest_msg.name.push_back("chest_height");
        chest_msg.position.push_back(0.65);
        chest_msg.velocity.push_back(18000);
        chest_height_pub.publish(chest_msg);

        sensor_msgs::JointState left_arm_msg;
        left_arm_msg.header.stamp = ros::Time::now();
        for(int i=0;i<6;i++)
        {
            left_arm_msg.name.push_back(a_left_name[i]);
            left_arm_msg.position.push_back(a_left_position[i]);
        }
        for(size_t i = 0; i < left_arm_msg.name.size(); ++i)
        {
            left_arm_msg.velocity.push_back(2000);
        }
        left_arm_pub.publish(left_arm_msg);

        sensor_msgs::JointState right_arm_msg;
        right_arm_msg.header.stamp = ros::Time::now();
        for(int i=0;i<6;i++)
        {
            right_arm_msg.name.push_back(a_right_name[i]);
            right_arm_msg.position.push_back(a_right_position[i]);
        }
        for(size_t i = 0; i < right_arm_msg.name.size(); ++i)
        {
            right_arm_msg.velocity.push_back(2000);
        }
        right_arm_pub.publish(right_arm_msg);
      
        count = 0;
        nState = 2;
    }
    
    if(nState == 3)
    {
        ROS_WARN("到达 2 航点，做另一个动作");
        sensor_msgs::JointState chest_msg;
        chest_msg.header.stamp = ros::Time::now();
        chest_msg.name.push_back("chest_height");
        chest_msg.position.push_back(0.7);
        chest_msg.velocity.push_back(18000);
        chest_height_pub.publish(chest_msg);

        sensor_msgs::JointState left_arm_msg;
        left_arm_msg.header.stamp = ros::Time::now();
        for(int i=0;i<6;i++)
        {
            left_arm_msg.name.push_back(b_left_name[i]);
            left_arm_msg.position.push_back(b_left_position[i]);
        }
        for(size_t i = 0; i < left_arm_msg.name.size(); ++i)
        {
            left_arm_msg.velocity.push_back(2000);
        }
        left_arm_pub.publish(left_arm_msg);
        
        sensor_msgs::JointState right_arm_msg;
        right_arm_msg.header.stamp = ros::Time::now();
        for(int i=0;i<6;i++)
        {
            right_arm_msg.name.push_back(b_right_name[i]);
            right_arm_msg.position.push_back(b_right_position[i]);
        }
        for(size_t i = 0; i < right_arm_msg.name.size(); ++i)
        {
            right_arm_msg.velocity.push_back(2000);
        }
        right_arm_pub.publish(right_arm_msg);

        nState = 4;
        count = 0;
    }

}

void cmdCallback(const std_msgs::StringConstPtr& msg)
{
    if (msg->data == "go")
    {
        ROS_WARN("开始行动，去往 1 航点");
        std_msgs::String nav_msg;
        nav_msg.data = "1";
        nState = 1;
        nav_pub.publish(nav_msg);
    }
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "demo_map_tool");

    init();

    ros::NodeHandle n;
    nav_pub = n.advertise<std_msgs::String>("/waterplus/navi_waypoint", 10);
    ros::Subscriber res_sub = n.subscribe("/waterplus/navi_result", 10, NavResultCallback);
    chest_height_pub = n.advertise<sensor_msgs::JointState>("/wpr2/chest_height", 10);
    left_arm_pub = n.advertise<sensor_msgs::JointState>("/wpr2/left_arm", 10);
    right_arm_pub = n.advertise<sensor_msgs::JointState>("/wpr2/right_arm", 10);
    ros::Subscriber cmd_sub = n.subscribe("/wpr2_cmd", 10, cmdCallback);

    sleep(1);
    
    ros::Rate rate(10);

    while(ros::ok())
    {
        if(nState == 2)
        {
            count ++;
            if(count > 50)
            {
                ROS_WARN("暂停完毕，去往 2 航点");
                std_msgs::String nav_msg;
                nav_msg.data = "2";
                nState = 3;
                count = 0;
                nav_pub.publish(nav_msg);
            }
        }

        if(nState == 4)
        {
            count ++;
            if(count > 50)
            {
                ROS_WARN("暂停完毕，去往 1 航点");
                std_msgs::String nav_msg;
                nav_msg.data = "1";
                nState = 1;
                count = 0;
                nav_pub.publish(nav_msg);
            }
        }
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}