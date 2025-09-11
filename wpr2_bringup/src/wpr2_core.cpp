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
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include "driver/wpr2_driver.h"
#include <math.h>

// 定义最大线加速度和角加速度
const double MAX_LINEAR_STEP = 0.2;  // 单位: m/s/loop，可根据需要调整
const double MAX_ANGULAR_STEP = 0.5; // 单位: rad/s/loop，可根据需要调整
static geometry_msgs::Twist curVel; // 用于存储当前实际发送给硬件的速度
static geometry_msgs::Twist targetVel; // 用于存储从 /cmd_vel 接收到的目标速度

static CWPR2_driver wpr2;
static int nLastMotorPos[4];
static int arLeftArmPos[6];
static int arLeftArmSpeed[6];
static int nLeftGripperPos;
static int nLeftGripperSpeed;
static int arRightArmPos[6];
static int arRightArmSpeed[6];
static int nRightGripperPos;
static int nRightGripperSpeed;

void accelerateControl()
{
    // 定义一个足够小的数，用于浮点数比较
    const double epsilon = 1e-6;

    // 检查当前速度是否已经足够接近目标速度
    if (std::abs(targetVel.linear.x - curVel.linear.x) < epsilon &&
        std::abs(targetVel.linear.y - curVel.linear.y) < epsilon &&
        std::abs(targetVel.angular.z - curVel.angular.z) < epsilon)
    {
        // 如果速度相等，则直接返回，不发送新的速度指令
        return;
    }

    // --- X方向线速度 ---
    double delta_vx = targetVel.linear.x - curVel.linear.x;
    if (delta_vx > MAX_LINEAR_STEP) {
        curVel.linear.x += MAX_LINEAR_STEP;
    } else if (delta_vx < -MAX_LINEAR_STEP) {
        curVel.linear.x -= MAX_LINEAR_STEP;
    } else {
        curVel.linear.x = targetVel.linear.x;
    }

    // --- Y方向线速度 ---
    double delta_vy = targetVel.linear.y - curVel.linear.y;
    if (delta_vy > MAX_LINEAR_STEP) {
        curVel.linear.y += MAX_LINEAR_STEP;
    } else if (delta_vy < -MAX_LINEAR_STEP) {
        curVel.linear.y -= MAX_LINEAR_STEP;
    } else {
        curVel.linear.y = targetVel.linear.y;
    }

    // --- Z方向角速度 ---
    double delta_wz = targetVel.angular.z - curVel.angular.z;
    if (delta_wz > MAX_ANGULAR_STEP) {
        curVel.angular.z += MAX_ANGULAR_STEP;
    } else if (delta_wz < -MAX_ANGULAR_STEP) {
        curVel.angular.z -= MAX_ANGULAR_STEP;
    } else {
        curVel.angular.z = targetVel.angular.z;
    }

    // 将经过步长限制后的速度发送给底层驱动
    wpr2.Velocity(curVel.linear.x, curVel.linear.y, curVel.angular.z);
}

void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //ROS_INFO("[wpr2] liner(%.2f %.2f) angular(%.2f)", msg->linear.x,msg->linear.y,msg->angular.z);
    // 更新目标速度值
    targetVel.linear.x = msg->linear.x;
    targetVel.linear.y = msg->linear.y;
    targetVel.angular.z = msg->angular.z;
}

static double kAngleToDegree = 18000/3.1415926;
void LeftArmCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //左手臂
    int nNumJoint = msg->position.size();
    if(nNumJoint > 6) nNumJoint = 6;
    for(int i=0;i<nNumJoint;i++)
    {
        ROS_WARN("[LeftArm 接收] msg->position[%d] = (%.2f)",i, msg->position[i]);
        arLeftArmPos[i] = msg->position[i] * kAngleToDegree;
        arLeftArmSpeed[i] = msg->velocity[i];
    }
    wpr2.SetLeftArm(arLeftArmPos,arLeftArmSpeed);
}

void LeftGripperCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //左手爪
    // int nGripperVal = 12000 - msg->position[0]*100*2000;
    // if(nGripperVal < 0)
    //     nGripperVal -= 1000;
    // if(nGripperVal < -4000)
    //     nGripperVal = -4000;
    // if(nGripperVal > 10000)
    //     nGripperVal = 10000;
    // nLeftGripperPos = nGripperVal;
    nLeftGripperPos = msg->position[0];
    nLeftGripperSpeed = msg->velocity[0];
    wpr2.SetLeftGripper(nLeftGripperPos,nLeftGripperSpeed);
}

void RightArmCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //右手臂
    int nNumJoint = msg->position.size();
    if(nNumJoint > 6) nNumJoint = 6;
    for(int i=0;i<nNumJoint;i++)
    {
        ROS_WARN("[RightArm 接收] msg->position[%d] = (%.2f)",i, msg->position[i]);
        arRightArmPos[i] = msg->position[i] * kAngleToDegree;
        arRightArmSpeed[i] = msg->velocity[i];
    }
    wpr2.SetRightArm(arRightArmPos,arRightArmSpeed);
}

void RightGripperCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //右手爪
    // int nGripperVal = 12000 - msg->position[0]*100*2000;
    // if(nGripperVal < 0)
    //     nGripperVal -= 1000;
    // if(nGripperVal < -4000)
    //     nGripperVal = -4000;
    // if(nGripperVal > 10000)
    //     nGripperVal = 10000;
    // nRightGripperPos = nGripperVal;
    nRightGripperPos = msg->position[0];
    nRightGripperSpeed = msg->velocity[0];
    wpr2.SetRightGripper(nRightGripperPos,nRightGripperSpeed);
}

void ChestHeightCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //脊柱升降
    int nNumJoint = msg->position.size();
    if(nNumJoint > 0) 
    {
        int nTorsoHeight = (0.7 - msg->position[0]) * 1000*1000;
        if(nTorsoHeight < 0) nTorsoHeight = 0;
        if(nTorsoHeight > 700000) nTorsoHeight = 700000;
        int nTorsoSpeed = msg->velocity[0];
        ROS_WARN("[ChestHeight 接收] msg->position[0] = (%.2f)",msg->position[0]);
        wpr2.SetTorsoHeight(nTorsoHeight,nTorsoSpeed);
    }
}

static geometry_msgs::Pose2D odom_delta_msg;
void OdomDeltaResetCallback(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("odom_delta reset");
    if( nFindIndex >= 0 )
    {
        odom_delta_msg.x = 0;
        odom_delta_msg.y = 0;
        odom_delta_msg.theta = 0;
        //ROS_WARN("[odom_delta reset]");
    }
}

// 麦克纳姆轮里程计参数
static double fKMVx =0.091f/(1990*2*4.5);
static double fKMVy = 0.095f/(2200*2*4.4);
static double fKMVz = 0.091f/(832*4*4.84);
// 全向里程计参数
static float fKOVx = 1.0f/sqrt(3.0f);
static float fKOVy = 2.0f/3.0f;
static float fKOVz = 1.0f/3.0f;

static float fSumX =0;
static float fSumY =0;
static float fSumZ =0;
static float fOdomX =0;
static float fOdomY =0;
static float fOdomZ =0;
static double fJointPosK = 0.01*M_PI/180;
static double fWheelPosK = -0.1*M_PI/180;
static geometry_msgs::Pose2D lastPose;
static geometry_msgs::Twist lastVel;
int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"wpr2_core");
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel",10,&CmdVelCallback);
    ros::Subscriber left_arm_sub = n.subscribe("wpr2/left_arm",10,&LeftArmCallback);
    ros::Subscriber left_gripper_sub = n.subscribe("wpr2/left_gripper",10,&LeftGripperCallback);
    ros::Subscriber right_arm_sub = n.subscribe("wpr2/right_arm",10,&RightArmCallback);
    ros::Subscriber right_gripper_sub = n.subscribe("wpr2/right_gripper",10,&RightGripperCallback);
    ros::Subscriber chest_height_sub = n.subscribe("wpr2/chest_height",10,&ChestHeightCallback);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu >("imu/data_raw", 10);

    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/dev/ftdi");
    wpr2.Open(strSerialPort.c_str(),115200);

    bool bImu;
    n_param.param<bool>("imu_pub", bImu, false);
    if(bImu == true)
        ROS_WARN("[wpr2_core] 开启姿态IMU信息发布！");
    
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states",10);
    sensor_msgs::JointState joint_msg;
    std::vector<std::string> joint_name(21);
    std::vector<double> joint_pos(21);

    joint_name[0] = "front_left_wheel_joint";
    joint_name[1] = "front_right_wheel_joint";
    joint_name[2] = "back_right_wheel_joint";
    joint_name[3] = "back_left_wheel_joint";
    joint_name[4] = "chest_height";
    joint_name[5] = "left_joint_1";
    joint_name[6] = "left_joint_2";
    joint_name[7] = "left_joint_3";
    joint_name[8] = "left_joint_4";
    joint_name[9] = "left_joint_5";
    joint_name[10] = "left_joint_6";
    joint_name[11] = "left_joint_finger";

    joint_name[12] = "right_joint_1";
    joint_name[13] = "right_joint_2";
    joint_name[14] = "right_joint_3";
    joint_name[15] = "right_joint_4";
    joint_name[16] = "right_joint_5";
    joint_name[17] = "right_joint_6";
    joint_name[18] = "right_joint_finger";

    joint_name[19] = "femto_height";
    joint_name[20] = "femto_pitch";

    for(int i=0;i<21;i++)
    {
        joint_pos[i] = 0.0f;
    }
    joint_pos[20] = -0.3f;
    n_param.getParam("zeros/femto_height", joint_pos[19]);
    n_param.getParam("zeros/femto_pitch", joint_pos[20]);

    for(int i=0;i<6;i++)
    {
        arLeftArmPos[i] = 0;
        arLeftArmSpeed[i] = 2000;
        arRightArmPos[i] = 0;
        arRightArmSpeed[i] = 2000;
    }

    ros::Publisher odom_pub;
    geometry_msgs::TransformStamped odom_trans;
    tf::TransformBroadcaster broadcaster;
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat;

    odom_pub = n.advertise<nav_msgs::Odometry>("odom",10);
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;

    ros::Subscriber ctrl_sub = n.subscribe("odom_delta_reset",10,&OdomDeltaResetCallback);
    ros::Publisher odom_delta_pub = n.advertise<geometry_msgs::Pose2D>("odom_delta",1);
    odom_delta_msg.x = 0;
    odom_delta_msg.y = 0;
    odom_delta_msg.theta = 0;

    // 初始化 curVel 和 targetVel
    curVel.linear.x = curVel.linear.y = curVel.linear.z = 0;
    curVel.angular.x = curVel.angular.y = curVel.angular.z = 0;
    targetVel = curVel;

    lastPose.x = lastPose.y = lastPose.theta = 0;
    lastVel.linear.x = lastVel.linear.y = lastVel.linear.z = lastVel.angular.x = lastVel.angular.y = lastVel.angular.z = 0;
    nLastMotorPos[0] = nLastMotorPos[1] = nLastMotorPos[2] = nLastMotorPos[3] = 0;

    ros::Rate r(20.0);
    while(n.ok())
    {
        // 加速度控制
        accelerateControl();
        // 关节运动
        wpr2.JointAction();
        // 读取底盘电路返回数据
        wpr2.ReadNewData();
        // 太长时间没有返回值，认为底盘断电，清零里程计，避免上电后里程计跳动过大
        wpr2.nParseCount ++;
        //ROS_INFO("[wpr2.nParseCount]= %d",wpr2.nParseCount);
        if(wpr2.nParseCount > 100)
        {
            wpr2.arMotorPos[0] =0; nLastMotorPos[0] = 0;
            wpr2.arMotorPos[1] =0; nLastMotorPos[1] = 0;
            wpr2.arMotorPos[2] =0; nLastMotorPos[2] = 0;
            wpr2.arMotorPos[3] =0; nLastMotorPos[3] = 0;
            wpr2.nParseCount = 0;
            curVel.linear.x = curVel.linear.y = curVel.linear.z = 0;
            curVel.angular.x = curVel.angular.y = curVel.angular.z = 0;
            targetVel = curVel;
            //ROS_INFO("empty");
        }
        
        last_time = current_time;
        current_time = ros::Time::now();

        double fVx,fVy,fVz;
        double fPosDiff[4];
        if(nLastMotorPos[0] != wpr2.arMotorPos[0] || nLastMotorPos[1] != wpr2.arMotorPos[1] || nLastMotorPos[2] != wpr2.arMotorPos[2] || nLastMotorPos[3] != wpr2.arMotorPos[3])
        //if(true)
        {
            //ROS_WARN("[M0] %d    [M1] %d    [M2] %d    [M3] %d",wpr2.arMotorPos[0],wpr2.arMotorPos[1],wpr2.arMotorPos[2],wpr2.arMotorPos[3]);
            fPosDiff[0] = (double)(wpr2.arMotorPos[0] - nLastMotorPos[0]); 
            fPosDiff[1] = (double)(wpr2.arMotorPos[1] - nLastMotorPos[1]);
            fPosDiff[2] = (double)(wpr2.arMotorPos[2] - nLastMotorPos[2]);
            fPosDiff[3] = (double)(wpr2.arMotorPos[3] - nLastMotorPos[3]);
            //ROS_INFO("[D0] %.2f    [D1] %.2f   [D2] %.2f    [D3] %.2f",fPosDiff[0],fPosDiff[1],fPosDiff[2],fPosDiff[3]);
            int nMaxDiff = 1000;
            if(
                fabs( fPosDiff[0] ) > nMaxDiff || fabs( fPosDiff[1] ) > nMaxDiff || fabs( fPosDiff[2] ) > nMaxDiff || fabs( fPosDiff[3] ) > nMaxDiff 
            )
            {
                // ROS_WARN("[M0]= %d    last =  %d",wpr2.arMotorPos[0],nLastMotorPos[0]); 
                // ROS_WARN("[M1]= %d    last =  %d",wpr2.arMotorPos[1],nLastMotorPos[1]); 
                // ROS_WARN("[M2]= %d    last =  %d",wpr2.arMotorPos[2],nLastMotorPos[2]); 
                // ROS_WARN("[M3]= %d    last =  %d",wpr2.arMotorPos[3],nLastMotorPos[3]);  
                // ROS_WARN("-----------------------------------"); 
            }

            //方案一 通过电机码盘解算里程计数据（位移/时间=速度）
            double fTimeDur = current_time.toSec() - last_time.toSec();
            if(wpr2.nBaseMode == MODE_MECANUM)
            {
                // 麦克纳姆轮里程计
                fVx = (fPosDiff[1] - fPosDiff[0]) * fKMVx;
                fVy = (fPosDiff[1] - fPosDiff[2]) *fKMVy;
                fVz = (fPosDiff[0] + fPosDiff[1] + fPosDiff[2] + fPosDiff[3])*fKMVz;
                fVx = fVx/(fTimeDur);
                fVy = fVy/(fTimeDur);
                fVz = fVz/(fTimeDur);
            }
            if(wpr2.nBaseMode == MODE_OMNI)
            {
                fVx = (fPosDiff[1] - fPosDiff[0]) * fKOVx;
                fVy = (fPosDiff[0] + fPosDiff[1]) - (fPosDiff[0] + fPosDiff[1] + fPosDiff[2])*fKOVy;
                fVz = (fPosDiff[0] + fPosDiff[1] + fPosDiff[2])*fKOVz;
                fVx = fVx/(fTimeDur*9100);
                fVy = fVy/(fTimeDur*9100);
                fVz = fVz/(fTimeDur*1840);
            }
            //ROS_WARN("[Velocity]    fVx=%.2f   fVy=%.2f    fVz=%.2f",fVx,fVy,fVz);
            // 方案二 直接把下发速度当作里程计积分依据
            // fVx = lastVel.linear.x;
            // fVy = lastVel.linear.y;
            // fVz = lastVel.angular.z;
            
            double dx = (lastVel.linear.x*cos(lastPose.theta) - lastVel.linear.y*sin(lastPose.theta))*fTimeDur;
            double dy = (lastVel.linear.x*sin(lastPose.theta) + lastVel.linear.y*cos(lastPose.theta))*fTimeDur;

            lastPose.x += dx;
            lastPose.y += dy;
            lastPose.theta += (fVz*fTimeDur);

            double pd_dx = (lastVel.linear.x*cos(odom_delta_msg.theta) - lastVel.linear.y*sin(odom_delta_msg.theta))*fTimeDur;
            double pd_dy = (lastVel.linear.x*sin(odom_delta_msg.theta) + lastVel.linear.y*cos(odom_delta_msg.theta))*fTimeDur;
            odom_delta_msg.x += pd_dx;
            odom_delta_msg.y += pd_dy;
            odom_delta_msg.theta += (fVz*fTimeDur);

            odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,lastPose.theta);
            //updata transform
            odom_trans.header.stamp = current_time;
            odom_trans.transform.translation.x = lastPose.x;
            odom_trans.transform.translation.y = lastPose.y;
            odom_trans.transform.translation.z = 0;
            odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(lastPose.theta);

            //filling the odometry
            odom.header.stamp = current_time;
            //position
            odom.pose.pose.position.x = lastPose.x;
            odom.pose.pose.position.y = lastPose.y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;
            //velocity
            odom.twist.twist.linear.x = fVx;
            odom.twist.twist.linear.y = fVy;
            odom.twist.twist.linear.z = 0;
            odom.twist.twist.angular.x = 0;
            odom.twist.twist.angular.y = 0;
            odom.twist.twist.angular.z = fVz;

            //plublishing the odometry and new tf
            broadcaster.sendTransform(odom_trans);
            odom_pub.publish(odom);

            lastVel.linear.x = fVx;
            lastVel.linear.y = fVy;
            lastVel.angular.z = fVz;

            nLastMotorPos[0] = wpr2.arMotorPos[0];
            nLastMotorPos[1] = wpr2.arMotorPos[1];
            nLastMotorPos[2] = wpr2.arMotorPos[2];
            nLastMotorPos[3] = wpr2.arMotorPos[3];
        }
        else
        {
            odom_trans.header.stamp = ros::Time::now();
            //plublishing the odometry and new tf
            broadcaster.sendTransform(odom_trans);
            odom.header.stamp = ros::Time::now();
            odom_pub.publish(odom);
            //ROS_INFO("[odom] zero");
        }

        odom_delta_pub.publish(odom_delta_msg);
        //ROS_INFO("[odom_delta_msg] x= %.2f  y=%.2f  th= %.2f", odom_delta_msg.x,odom_delta_msg.y,odom_delta_msg.theta);
    
        if(bImu == true)
        {
            //imu
            sensor_msgs::Imu imu_msg = sensor_msgs::Imu();	
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "imu";
            imu_msg.orientation.x = wpr2.fQuatX;
            imu_msg.orientation.y = wpr2.fQuatY;
            imu_msg.orientation.z = wpr2.fQuatZ;
            imu_msg.orientation.w = wpr2.fQuatW;

            imu_msg.angular_velocity.x = wpr2.fGyroX;
            imu_msg.angular_velocity.y = wpr2.fGyroY;
            imu_msg.angular_velocity.z = wpr2.fGyroZ;

            imu_msg.linear_acceleration.x = wpr2.fAccX;
            imu_msg.linear_acceleration.y = wpr2.fAccY;
            imu_msg.linear_acceleration.z = wpr2.fAccZ;

            imu_pub.publish(imu_msg);
        }

        // wheels tf
        for(int i=0;i<4;i++)
        {
              joint_pos[i] = wpr2.arMotorPos[i]*fWheelPosK;
        }

        // joint tf
        joint_msg.header.stamp = ros::Time::now();
        joint_msg.header.seq ++;
        for(int i=1;i<15;i++)
        {
            joint_pos[i+4] = wpr2.arJointPosRecv[i]*fJointPosK;
            // ROS_WARN("[tf 发送] msg->position[%d] name=%s  = (%.2f)",i+4, joint_name[i+4].c_str() ,joint_pos[i+4]);
        }
        // torso
        joint_pos[4] = wpr2.arJointPosRecv[0];
        joint_pos[4] *= 0.000001;
        joint_pos[4] = 0.7-joint_pos[4];
        
        // gripper
        // joint_pos[11]  = 0.02 - (float)(wpr2.arJointPosRecv[7]+2000)/400000;
        // joint_pos[18]  = 0.02 - (float)(wpr2.arJointPosRecv[14]+2000)/400000;

        joint_msg.name = joint_name;
        joint_msg.position = joint_pos;
        joint_state_pub.publish(joint_msg);
        // ROS_WARN("Joint pos pub");

        ros::spinOnce();
        r.sleep();
    }
}
