#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h> 
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <cmath>

// 抓取参数调节（单位：米）
static float grab_y_offset = -0.02f;        //抓取前，对准物品，机器人的横向位移偏移量
static float grab_lift_offset = 0.0f;       //脊柱高度的补偿偏移量
static float grab_forward_offset = 0.0f;    //手臂抬起后，机器人向前抓取物品移动的位移偏移量
static float grab_gripper_value = 0.3;      //抓取物品时，手爪闭合后的指头间距
static int joint_speed = 1000;

#define STEP_WAIT           0
#define STEP_FIND_OBJ       1
#define STEP_ALIGN_OBJ      2
#define STEP_GRAB           3
#define STEP_BACKWARD       4
#define STEP_DONE           5

static int nStep = STEP_FIND_OBJ;
static std::string pc_topic;
static ros::Publisher vel_pub;
static ros::Publisher arm_ctrl_pub;
static sensor_msgs::JointState arm_ctrl_msg;
static ros::Publisher gripper_ctrl_pub;
static sensor_msgs::JointState gripper_ctrl_msg;
static ros::Publisher result_pub;
static ros::Publisher right_grab_pub;

void VelCmd(float inVx , float inVy, float inTz);


static std_msgs::String result_msg;

static ros::Publisher odom_ctrl_pub;
static std_msgs::String ctrl_msg;
static geometry_msgs::Pose2D odom_delta;

static float fObjGrabX = 0;
static float fObjGrabY = 0;
static float fObjGrabZ = 0;
static float fMoveTargetX = 0;
static float fMoveTargetY = 0;

static int nObjectCounter = 0;
static int nTimeDelayCounter = 0;

static float fTargetGrabX = 0.9;        //抓取时目标物品的x坐标
static float fTargetGrabY = 0.0;        //抓取时目标物品的y坐标

// 用于滤波的全局变量
std::vector<geometry_msgs::Point> recent_points;
const int REQUIRED_CONSECUTIVE_POINTS = 5;      // 需要连续检测到的次数
const double DISTANCE_THRESHOLD = 0.05;         // 连续检测点的距离阈值（米）

void GrabActionCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // 目标物品的坐标
    fObjGrabX = msg->position.x;
    fObjGrabY = msg->position.y;
    fObjGrabZ = msg->position.z;
    ROS_WARN("[OBJ_TO_GRAB] x = %.2f y= %.2f ,z= %.2f " ,fObjGrabX, fObjGrabY, fObjGrabZ);
    ctrl_msg.data = "odom_delta reset";
    odom_ctrl_pub.publish(ctrl_msg);

    // ajudge the dist to obj
    fMoveTargetX = fObjGrabX - fTargetGrabX;
    fMoveTargetY = fObjGrabY - fTargetGrabY + grab_y_offset;
    ROS_WARN("[MOVE_TARGET] x = %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);
    nStep = STEP_ALIGN_OBJ;
}

// 订阅/yolo_target_3d_point的回调函数，用于滤波和触发抓取
void YoloPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // 只在寻找物体步骤(STEP_FIND_OBJ)时执行滤波逻辑
    if (nStep != STEP_FIND_OBJ)
    {
        // 如果不在寻找状态，清空历史数据，以防过时数据干扰
        if (!recent_points.empty()) {
            recent_points.clear();
        }
        return;
    }

    geometry_msgs::Point current_point = msg->point;
    ROS_INFO("接收到目标点: (%.2f, %.2f, %.2f)", current_point.x, current_point.y, current_point.z);

    if (recent_points.empty())
    {
        // 收到序列中的第一个点
        recent_points.push_back(current_point);
    }
    else
    {
        // 和序列中最后一个点进行比较
        const auto& last_point = recent_points.back();
        double dist = std::sqrt(std::pow(current_point.x - last_point.x, 2) +
                                std::pow(current_point.y - last_point.y, 2) +
                                std::pow(current_point.z - last_point.z, 2));

        if (dist <= DISTANCE_THRESHOLD)
        {
            // 距离在阈值内，是有效点，加入序列
            recent_points.push_back(current_point);
            ROS_INFO("近距点增加，当前点数量: %zu", recent_points.size());
        }
        else
        {
            // 距离太远，说明目标不稳定或检测到新目标，重置序列
            ROS_INFO("新点距离太远 (距离=%.3f m). 重新开始计数", dist);
            recent_points.clear();
            recent_points.push_back(current_point);
        }
    }

    // 检查是否已收集到足够数量的连续点
    if (recent_points.size() >= REQUIRED_CONSECUTIVE_POINTS)
    {
        ROS_INFO("Target confirmed with %d consecutive points.", (int)recent_points.size());

        // 为了更稳定，计算所有点的平均位置
        double avg_x = 0, avg_y = 0, avg_z = 0;
        for (const auto& pt : recent_points)
        {
            avg_x += pt.x;
            avg_y += pt.y;
            avg_z += pt.z;
        }
        fObjGrabX = avg_x / recent_points.size();
        fObjGrabY = avg_y / recent_points.size();
        fObjGrabZ = avg_z / recent_points.size();

        ROS_WARN("[OBJ_TO_GRAB] 抓取物品坐标: x=%.2f, y=%.2f, z=%.2f", fObjGrabX, fObjGrabY, fObjGrabZ);

        // 重置里程计增量
        ctrl_msg.data = "odom_delta reset";
        odom_ctrl_pub.publish(ctrl_msg);

        // 计算机器人需要移动的目标位置
        fMoveTargetX = fObjGrabX - fTargetGrabX;
        fMoveTargetY = fObjGrabY - fTargetGrabY + grab_y_offset;
        ROS_WARN("[MOVE_TARGET] 平移对准坐标 x=%.2f, y=%.2f", fMoveTargetX, fMoveTargetY);
        
        // 切换到下一个步骤：对准物体
        nStep = STEP_ALIGN_OBJ;
        
        // 清空数据，为下一次抓取做准备
        recent_points.clear();
    }
}


void OdomDeltaCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    odom_delta.x = msg->x;
    odom_delta.y = msg->y;
    odom_delta.theta = msg->theta;
}

void VelCmd(float inVx , float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = inVx;
    vel_cmd.linear.y = inVy;
    vel_cmd.angular.z = inTz;
    vel_pub.publish(vel_cmd);
}

void BehaviorCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = msg->data.find("grab stop");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[grab_stop] ");
        nStep = STEP_WAIT;
        geometry_msgs::Twist vel_cmd;
        vel_cmd.linear.x = 0;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = 0;
        vel_pub.publish(vel_cmd);
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpr2_grab_bottle");
    ROS_WARN("wpr2_grab_bottle start!");

    ros::NodeHandle nh;

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    arm_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpr2/right_arm",10);
    gripper_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpr2/right_gripper",10);
    result_pub = nh.advertise<std_msgs::String>("/wpr2/grab_result", 10);
    right_grab_pub = nh.advertise<std_msgs::Float32>("/wpr2/right_grab", 10); 
    ros::Subscriber sub_yolo_point = nh.subscribe("/yolo_target_3d_point", 10, YoloPointCallback);

    ros::Subscriber sub_grab_pose = nh.subscribe("/wpr2/grab_bottle", 1, GrabActionCallback);
    ros::Subscriber sub_beh = nh.subscribe("/wpr2/behaviors", 10, BehaviorCB);
    odom_ctrl_pub = nh.advertise<std_msgs::String>("odom_delta_reset", 10);
    ros::Subscriber odom_delta_sub = nh.subscribe("odom_delta", 1, OdomDeltaCallback);

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
    arm_ctrl_msg.velocity[1] = joint_speed;
    arm_ctrl_msg.velocity[2] = joint_speed;
    arm_ctrl_msg.velocity[3] = joint_speed;
    arm_ctrl_msg.velocity[4] = joint_speed;
    arm_ctrl_msg.velocity[5] = joint_speed;

    gripper_ctrl_msg.name.resize(1);
    gripper_ctrl_msg.position.resize(1);
    gripper_ctrl_msg.velocity.resize(1);
    gripper_ctrl_msg.name[0] = "right_gripper";
    gripper_ctrl_msg.position[0] = 0.1;
    gripper_ctrl_msg.velocity[0] = 2000;

    ros::Rate r(30);
    while(nh.ok())
    {
        //1、对物品位置进行滤波
        if(nStep == STEP_FIND_OBJ)
        {
            // 滤波逻辑移至YoloPointCallback中，由消息回调驱动
            // 主循环此处保持空闲，等待状态切换
            VelCmd(0,0,0); // 等待时停止移动
        }
    
        //4、左右平移对准目标物品
        if(nStep == STEP_ALIGN_OBJ)
        {
            float vx,vy;
            vx = (fMoveTargetX - odom_delta.x)/2;
            vy = (fMoveTargetY - odom_delta.y)/2;

            VelCmd(vx,vy,0);
            //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, odom_delta.x ,odom_delta.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                ctrl_msg.data = "odom_delta reset";
                odom_ctrl_pub.publish(ctrl_msg);
                nTimeDelayCounter = 0;

                // ****** 新增代码段开始 ******
                std_msgs::Float32 right_grab_msg;
                right_grab_msg.data = 0.05;
                right_grab_pub.publish(right_grab_msg);
                // ****** 新增代码段结束 ******

                nStep = STEP_GRAB;
            }

            result_msg.data = "object x";
            result_pub.publish(result_msg);
        }

       
        //7、抓取物品
        if(nStep == STEP_GRAB)
        {
            if(nTimeDelayCounter == 0)
            {
                result_msg.data = "grab";
                result_pub.publish(result_msg);
            }
           
            VelCmd(0,0,0);
            if(nTimeDelayCounter > 3*30)
            {
                nTimeDelayCounter = 0;
                ROS_WARN("[STEP_OBJ_UP]");
                nStep = STEP_BACKWARD;
            }
        }

        //9、带着物品后退
        if(nStep == STEP_BACKWARD)
        {
            //ROS_WARN("[STEP_BACKWARD] nTimeDelayCounter = %d " ,nTimeDelayCounter);
            //nTimeDelayCounter++;
            
            float vx,vy;
            vx = (fMoveTargetX - odom_delta.x)/2;

            if(odom_delta.x > -0.3)
                vy = 0;
            else
                vy = (fMoveTargetY - odom_delta.y)/2;

            VelCmd(vx,vy,0);

            //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, odom_delta.x ,odom_delta.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                ctrl_msg.data = "odom_delta reset";
                odom_ctrl_pub.publish(ctrl_msg);
                nTimeDelayCounter = 0;
                ROS_WARN("[STEP_DONE]");
                nStep = STEP_DONE;
            }

            result_msg.data = "backward";
            result_pub.publish(result_msg);
        }

        //10、抓取任务完毕
        if(nStep == STEP_DONE)
        {
            if(nTimeDelayCounter < 10)
            {
                VelCmd(0,0,0);
                nTimeDelayCounter ++;
                result_msg.data = "done";
                result_pub.publish(result_msg);
            }
            else
            {
                nStep = STEP_WAIT;
            }
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}