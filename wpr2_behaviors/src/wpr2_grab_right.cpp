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
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

#define STEP_WAIT           0
#define STEP_HAND_UP        1
#define STEP_FORWARD        2
#define STEP_REACH          3
#define STEP_GRAB           4
#define STEP_OBJ_UP         5
#define STEP_DONE           7
static int nStep = STEP_WAIT;

static ros::Publisher arm_ctrl_pub;
static sensor_msgs::JointState arm_ctrl_msg;
static ros::Publisher gripper_ctrl_pub;
static sensor_msgs::JointState gripper_ctrl_msg;
static float gripper_value = 0.15;

static ros::Publisher grab_result_pub;
static std_msgs::String result_msg;

static int nTimeDelayCounter = 0;
static int joint_speed = 2000;
static float right_position[6];
static bool joints_arrived = false;

void ArmAction()
{
    for(int i=0;i<6;i++)
        arm_ctrl_msg.position[i] = right_position[i];
    arm_ctrl_pub.publish(arm_ctrl_msg);
    joints_arrived = false;
}

void RightGrabCallback(const std_msgs::Float32::ConstPtr& msg)
{
    gripper_value = msg->data;

    right_position[0] = 0.7846;
    right_position[1] = -0.5484;
    right_position[2] = -0.9424;
    right_position[3] = 1.6701;
    right_position[4] = 1.9939;
    right_position[5] = 1.1738;
    ArmAction();

    gripper_ctrl_msg.position[0] = 0.15;
    gripper_ctrl_pub.publish(gripper_ctrl_msg);

    nStep = STEP_HAND_UP;

    ROS_INFO("[wpr2_right_grab] --1-- 开始抬手");
}

void JointsResultCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = msg->data.find("done");
    if( nFindIndex >= 0 )
    {
        ROS_INFO("[wpr2_right_grab] 接收到关节运动结果 done");
        joints_arrived = true;
    }

}

void BehaviorCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = msg->data.find("grab stop");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[grab_stop] ");
        
    }

}

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "wpr2_right_grab");
    ROS_INFO("wpr2_right_grab start!");

    ros::NodeHandle nh;

    arm_ctrl_pub = nh.advertise<sensor_msgs::JointState>("wpr2/right_arm", 10);
    gripper_ctrl_pub = nh.advertise<sensor_msgs::JointState>("wpr2/right_gripper", 10);
    grab_result_pub = nh.advertise<std_msgs::String>("/wpr2/grab_result", 10);

    ros::Subscriber sub_grab_pose = nh.subscribe("/wpr2/right_grab", 10, RightGrabCallback);
    ros::Subscriber sub_beh = nh.subscribe("/wpr2/behaviors", 10, BehaviorCB);
    ros::Subscriber sub_joints_result = nh.subscribe("wpr2/joints_result", 10, JointsResultCB);

    arm_ctrl_msg.name.resize(6);
    arm_ctrl_msg.position.resize(6);
    arm_ctrl_msg.velocity.resize(6);
    arm_ctrl_msg.name[0] = "right_joint_1";
    arm_ctrl_msg.name[1] = "right_joint_2";
    arm_ctrl_msg.name[2] = "right_joint_3";
    arm_ctrl_msg.name[3] = "right_joint_4";
    arm_ctrl_msg.name[4] = "right_joint_5";
    arm_ctrl_msg.name[5] = "right_joint_6";
    arm_ctrl_msg.position[0] = -0.6;
    arm_ctrl_msg.position[1] = 1.57;
    arm_ctrl_msg.position[2] = 0;
    arm_ctrl_msg.position[3] = 0;
    arm_ctrl_msg.position[4] = 1.0;
    arm_ctrl_msg.position[5] = 0;
    arm_ctrl_msg.velocity[0] = joint_speed;
    arm_ctrl_msg.velocity[1] = 2000;
    arm_ctrl_msg.velocity[2] = joint_speed;
    arm_ctrl_msg.velocity[3] = 8000;
    arm_ctrl_msg.velocity[4] = joint_speed;
    arm_ctrl_msg.velocity[5] = 5000;

    gripper_ctrl_msg.name.resize(1);
    gripper_ctrl_msg.position.resize(1);
    gripper_ctrl_msg.velocity.resize(1);
    gripper_ctrl_msg.name[0] = "right_gripper";
    gripper_ctrl_msg.position[0] = 0.15;
    gripper_ctrl_msg.velocity[0] = 2000;

    ros::Rate r(30);
    while(nh.ok())
    {

        //1、抬起手臂
        if(nStep == STEP_HAND_UP)
        {
            if(joints_arrived == true)
            {
                ROS_INFO("[wpr2_right_grab] --2-- 抬手完毕，往前伸手!");

                right_position[0] = 1.3069;
                right_position[1] = 1.0160;
                right_position[2] = -1.4946;
                right_position[3] = 1.7675;
                right_position[4] = 1.4017;
                right_position[5] = -0.8038;
                ArmAction();

                nStep = STEP_FORWARD;
            }
        }

        //2、前进靠近物品
        if(nStep == STEP_FORWARD)
        {
            if(joints_arrived == true)
            {
                ROS_INFO("[wpr2_right_grab] --3-- 伸手完毕，往前让物品进入夹爪!");

                right_position[0] = 1.3386;
                right_position[1] = 1.6764;
                right_position[2] = -1.2390;
                right_position[3] = 0.4614;
                right_position[4] = 1.1884;
                right_position[5] = -0.1357;
                ArmAction();

                nStep = STEP_REACH;
            }
        }

        //3、物品进手爪
        if(nStep == STEP_REACH)
        {
            if(joints_arrived == true)
            {
                ROS_INFO("[wpr1_right_grab] 关闭夹爪!");
                
                gripper_ctrl_msg.position[0] = gripper_value;
                gripper_ctrl_pub.publish(gripper_ctrl_msg);
                ArmAction();

                nStep = STEP_GRAB;
            }
        }

        //4、抓取物品
        if(nStep == STEP_GRAB)
        {
            if(joints_arrived == true)
            {
                ROS_INFO("[wpr1_right_grab] 抓取完成!向上抬伸");

                right_position[0] = 1.40;
                ArmAction();

                nStep = STEP_REACH;
            }
        }

        //5、带着物品抬起
        if(nStep == STEP_OBJ_UP)
        {
           if(joints_arrived == true)
            {
                ROS_INFO("[wpr1_right_grab] 物品抬升完成!");
                std_msgs::String grab_res_msg;
                grab_res_msg.data = "done";
                grab_result_pub.publish(grab_res_msg);

                nStep = STEP_DONE;
            }
        }

        //6、抓取任务完毕
        if(nStep == STEP_DONE)
        {
            
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}