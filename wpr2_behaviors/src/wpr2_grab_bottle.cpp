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
static float grab_y_offset = 0.0f;        //抓取前，对准物品，机器人的横向位移偏移量
static float grab_lift_offset = 0.0f;       //脊柱高度的补偿偏移量
static float grab_forward_range = 0.3f;    //手臂抬起后，机器人向前抓取物品移动的移动距离
static float grab_gripper_value = 0.06;      //抓取物品时，手爪闭合后的指头间距

// 用于滤波的全局变量
std::vector<geometry_msgs::Point> recent_points;
const int REQUIRED_CONSECUTIVE_POINTS = 30;      // 需要连续检测这么多次才能确认物品检测到了
const double DISTANCE_THRESHOLD = 0.01;         // 连续检测点的距离阈值（米）

static float fTargetGrabX = 0.7;        //抓取时目标物品的x坐标
static float fTargetGrabY = 0.0;        //抓取时目标物品的y坐标

static int joint_speed = 2000;

#define STEP_WAIT           0
#define STEP_INIT_POSE      1
#define STEP_FIND_OBJ       2
#define STEP_ALIGN_OBJ      3
#define STEP_HAND_UP        4
#define STEP_FORWARD        5
#define STEP_GRAB           6
#define STEP_OBJ_UP         7
#define STEP_BACKWARD       8
#define STEP_HOLDING        9
#define STEP_DONE           10

static int nStep = STEP_WAIT;
static std::string pc_topic;
static ros::Publisher vel_pub;
static ros::Publisher chest_height_pub;
static sensor_msgs::JointState chest_height_msg;
static ros::Publisher left_arm_pub;
static sensor_msgs::JointState left_arm_msg;
static ros::Publisher right_arm_pub;
static sensor_msgs::JointState right_arm_msg;
static ros::Publisher gripper_ctrl_pub;
static sensor_msgs::JointState gripper_ctrl_msg;
static ros::Publisher result_pub;
static ros::Publisher right_grab_pub;
static ros::Publisher yolo_switch_pub;
static std_msgs::String yolo_msg;

void VelCmd(float inVx , float inVy, float inTz);

static float right_position[6];
// 检测动作是否到位的机制
static bool joints_arrived = false;
void RightArmAction()
{
    for(int i=0;i<6;i++)
        right_arm_msg.position[i] = right_position[i];
    right_arm_pub.publish(right_arm_msg);
    joints_arrived = false;
}

void JointsResultCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = msg->data.find("done");
    if( nFindIndex >= 0 )
    {
        ROS_INFO("[wpr2_grab_bottle] 接收到关节运动结果 done");
        joints_arrived = true;
    }

}


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

void GrabActionCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // 目标物品的坐标
    // fObjGrabX = msg->position.x;
    // fObjGrabY = msg->position.y;
    // fObjGrabZ = msg->position.z;
    // ROS_WARN("[wpr2_grab_bottle] 抓取坐标 x = %.2f y= %.2f ,z= %.2f " ,fObjGrabX, fObjGrabY, fObjGrabZ);
    // ctrl_msg.data = "odom_delta reset";
    // odom_ctrl_pub.publish(ctrl_msg);

    // // ajudge the dist to obj
    // fMoveTargetX = fObjGrabX - fTargetGrabX;
    // fMoveTargetY = fObjGrabY - fTargetGrabY + grab_y_offset;
    // ROS_WARN("[wpr2_grab_bottle] 移动 x = %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);

    // 进入观测姿态
    right_position[0] = -0.6;
    right_position[1] = 1.57;
    right_position[2] = 0;
    right_position[3] = 1.5;
    right_position[4] = 0;
    right_position[5] = 0;
    RightArmAction();
    // 左臂是固定姿势
    left_arm_msg.position[0] = 0.6;
    left_arm_msg.position[1] = -1.57;
    left_arm_msg.position[2] = 0;
    left_arm_msg.position[3] = -1.5;
    left_arm_msg.position[4] = 0;
    left_arm_msg.position[5] = 0;
    left_arm_pub.publish(left_arm_msg);

    chest_height_msg.position[0] = 0.5;
    chest_height_pub.publish(chest_height_msg);

    // 张开手爪
    gripper_ctrl_msg.position[0] = 0.15;
    gripper_ctrl_pub.publish(gripper_ctrl_msg); 

    nStep = STEP_INIT_POSE;
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

    // 观测姿势没到位也不开始检测
    if(joints_arrived == false)
    {
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

        ROS_WARN("[wpr2_grab_bottle] 抓取物品坐标: x=%.2f, y=%.2f, z=%.2f", fObjGrabX, fObjGrabY, fObjGrabZ);

        // 关闭物品检测
        yolo_msg.data = "yolo stop";
        yolo_switch_pub.publish(yolo_msg);

        // 检查一下物品位置是否对准
        float diff_x = fObjGrabX - fTargetGrabX;
        float diff_y = fObjGrabY - fTargetGrabY + grab_y_offset;
        if(fabs(diff_x) < 0.03 && fabs(diff_y) < 0.03)
        {
            ROS_WARN("[wpr2_grab_bottle] 已经对准物品，开始抬手（误差 x=%.2f y=%.2f）",diff_x,diff_x);
            // 对准物品后开始抬手
            right_position[0] = 1.4813;
            right_position[1] = 1.5723;
            right_position[2] = -1.5426;
            right_position[3] = 1.3623;
            right_position[4] = 1.4895;
            right_position[5] = -1.3667;
            RightArmAction();
            ROS_INFO("[wpr2_right_bottle] --2-- 开始抬手!");

            nStep = STEP_HAND_UP;
            // 清空数据，为下一次抓取做准备
            recent_points.clear();
            return;
        }
        else
        {   // 没对准物品，继续对准
            
            // 重置里程计增量
            ctrl_msg.data = "odom_delta reset";
            odom_ctrl_pub.publish(ctrl_msg);

            // 计算机器人需要移动的目标位置
            fMoveTargetX = diff_x;
            fMoveTargetY = diff_y;
            ROS_WARN("[wpr2_grab_bottle] 平移对准坐标 x=%.2f, y=%.2f", fMoveTargetX, fMoveTargetY);

            chest_height_msg.position[0] = 0.7 - (1.1-fObjGrabZ) + grab_lift_offset;
            ROS_WARN("[wpr2_grab_bottle] 升降高度 = %.2f" ,chest_height_msg.position[0]);
            if(chest_height_msg.position[0] < 0.4) chest_height_msg.position[0] = 0.4; //保护一下，某些情况下去除
            chest_height_pub.publish(chest_height_msg);
            
            // 切换到下一个步骤：对准物体
            nStep = STEP_ALIGN_OBJ;
            
            // 清空数据，为下一次抓取做准备
            recent_points.clear();
        }
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

void RightResultCallback(const std_msgs::String::ConstPtr &msg)
{

    
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
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "wpr2_grab_bottle");
    ROS_WARN("wpr2_grab_bottle start!");

    ros::NodeHandle nh;

    yolo_switch_pub = nh.advertise<std_msgs::String>("/func_switch", 10);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    chest_height_pub = nh.advertise<sensor_msgs::JointState>("/wpr2/chest_height", 10);
    right_arm_pub = nh.advertise<sensor_msgs::JointState>("/wpr2/right_arm",10);
    left_arm_pub = nh.advertise<sensor_msgs::JointState>("/wpr2/left_arm",10);
    gripper_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpr2/right_gripper",10);
    ros::Subscriber sub_joints_result = nh.subscribe("wpr2/joints_result", 10, JointsResultCB);
    result_pub = nh.advertise<std_msgs::String>("/wpr2/grab_result", 10);
    right_grab_pub = nh.advertise<std_msgs::Float32>("/wpr2/right_grab", 10); 
    ros::Subscriber sub_yolo_point = nh.subscribe("/yolo_target_3d_point", 10, YoloPointCallback);

    ros::Subscriber sub_grab_pose = nh.subscribe("/wpr2/grab_bottle", 1, GrabActionCallback);
    ros::Subscriber sub_grab_right = nh.subscribe("/wpr2/right_result", 10, RightResultCallback);
    ros::Subscriber sub_beh = nh.subscribe("/wpr2/behaviors", 10, BehaviorCB);
    odom_ctrl_pub = nh.advertise<std_msgs::String>("odom_delta_reset", 10);
    ros::Subscriber odom_delta_sub = nh.subscribe("odom_delta", 1, OdomDeltaCallback);

    right_arm_msg.name.resize(6);
    right_arm_msg.position.resize(6);
    right_arm_msg.velocity.resize(6);
    right_arm_msg.name[0] = "right_joint_1";
    right_arm_msg.name[1] = "right_joint_2";
    right_arm_msg.name[2] = "right_joint_3";
    right_arm_msg.name[3] = "right_joint_4";
    right_arm_msg.name[4] = "right_joint_5";
    right_arm_msg.name[5] = "right_joint_6";
    right_arm_msg.position[0] = -0.6;
    right_arm_msg.position[1] = 1.57;
    right_arm_msg.position[2] = 0;
    right_arm_msg.position[3] = 1.5;
    right_arm_msg.position[4] = 0;
    right_arm_msg.position[5] = 0;
    right_arm_msg.velocity[0] = joint_speed;
    right_arm_msg.velocity[1] = joint_speed;
    right_arm_msg.velocity[2] = joint_speed;
    right_arm_msg.velocity[3] = joint_speed;
    right_arm_msg.velocity[4] = joint_speed;
    right_arm_msg.velocity[5] = joint_speed;

    left_arm_msg.name.resize(6);
    left_arm_msg.position.resize(6);
    left_arm_msg.velocity.resize(6);
    left_arm_msg.name[0] = "left_joint_1";
    left_arm_msg.name[1] = "left_joint_2";
    left_arm_msg.name[2] = "left_joint_3";
    left_arm_msg.name[3] = "left_joint_4";
    left_arm_msg.name[4] = "left_joint_5";
    left_arm_msg.name[5] = "left_joint_6";
    left_arm_msg.position[0] = 0.6;
    left_arm_msg.position[1] = -1.57;
    left_arm_msg.position[2] = 0;
    left_arm_msg.position[3] = -1.5;
    left_arm_msg.position[4] = 0;
    left_arm_msg.position[5] = 0;
    left_arm_msg.velocity[0] = joint_speed;
    left_arm_msg.velocity[1] = joint_speed;
    left_arm_msg.velocity[2] = joint_speed;
    left_arm_msg.velocity[3] = joint_speed;
    left_arm_msg.velocity[4] = joint_speed;
    left_arm_msg.velocity[5] = joint_speed;

    gripper_ctrl_msg.name.resize(1);
    gripper_ctrl_msg.position.resize(1);
    gripper_ctrl_msg.velocity.resize(1);
    gripper_ctrl_msg.name[0] = "right_gripper";
    gripper_ctrl_msg.position[0] = 0.1;
    gripper_ctrl_msg.velocity[0] = 2000;

    chest_height_msg.name.resize(1);
    chest_height_msg.position.resize(1);
    chest_height_msg.velocity.resize(1);
    chest_height_msg.name[0] = "chest_height";
    chest_height_msg.position[0] = 0.7;
    chest_height_msg.velocity[0] = 15000;

    sleep(1);

    ros::Rate r(30);
    while(nh.ok())
    {
        // 1、初始姿态
        if(nStep == STEP_INIT_POSE)
        {
            if(joints_arrived == true)
            {
                ROS_WARN("[wpr2_grab_bottle] 初始姿态完成，开始进行物品检测。");
                // 开启YOLO识别
                yolo_msg.data = "yolo start";
                yolo_switch_pub.publish(yolo_msg);
                nStep = STEP_FIND_OBJ;
                nTimeDelayCounter = 10 * 30; //开启识别倒计时，如果没找到物品就及时返回
                continue;
            }  
            VelCmd(0,0,0);
        }

        // 1、对物品位置进行滤波
        if(nStep == STEP_FIND_OBJ)
        {
            // 识别逻辑移至YoloPointCallback中，由消息回调驱动
            // 主循环此处进行超时判定 
            VelCmd(0,0,0);
            
            nTimeDelayCounter --;
            if(nTimeDelayCounter % 30 == 0)
                ROS_INFO("[wpr2_grab_bottle] 物品识别倒计时 ... %d",nTimeDelayCounter / 30);
            if( nTimeDelayCounter <= 0)
            {
                ROS_WARN("[wpr2_grab_bottle] 没识别到物品，及时返回 not found 结果！");
                result_msg.data = "not found";
                result_pub.publish(result_msg);
                nStep = STEP_DONE;
            }
        }
    
        // 2、左右平移对准目标物品
        if(nStep == STEP_ALIGN_OBJ)
        {
            float vx,vy;
            vx = (fMoveTargetX - odom_delta.x)/2;
            vy = (fMoveTargetY - odom_delta.y)/2;

            VelCmd(vx,vy,0);
            ROS_INFO("[对准物品] 目标(%.2f %.2f)  里程(%.2f , %.2f) 速度(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, odom_delta.x ,odom_delta.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                ctrl_msg.data = "odom_delta reset";
                odom_ctrl_pub.publish(ctrl_msg);

                ROS_WARN("[wpr2_grab_bottle] 移动完成后打开检测，再次进行物品对准确认。");

                // 再次开启物品检测
                yolo_msg.data = "yolo start";
                yolo_switch_pub.publish(yolo_msg);
                nStep = STEP_FIND_OBJ;
                nTimeDelayCounter = 5 * 30; //开启识别倒计时，如果没找到物品就及时返回
            }
        }

        // 3、抬手
        if(nStep == STEP_HAND_UP)
        {
            if(joints_arrived == true)
            {
                ROS_INFO("[wpr2_right_bottle] --3-- 抬手完毕，往前移动!");// 重置里程计增量
                ctrl_msg.data = "odom_delta reset";
                odom_ctrl_pub.publish(ctrl_msg);

                fMoveTargetX = grab_forward_range;
                fMoveTargetY = 0;
                nStep = STEP_FORWARD;
                continue;
            }
        }

        // 4、往前移动让物品进入手爪
        if(nStep == STEP_FORWARD)
        {
            float vx,vy;
            vx = (fMoveTargetX - odom_delta.x)/2;
            vy = (fMoveTargetY - odom_delta.y)/2;

            VelCmd(vx,vy,0);
            ROS_INFO("[前进] 目标(%.2f %.2f)  里程(%.2f , %.2f) 速度(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, odom_delta.x ,odom_delta.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                ctrl_msg.data = "odom_delta reset";
                odom_ctrl_pub.publish(ctrl_msg);
                nTimeDelayCounter = 0;

                // 闭合手爪
                gripper_ctrl_msg.position[0] = grab_gripper_value;
                gripper_ctrl_pub.publish(gripper_ctrl_msg);
                joints_arrived = false;

                nTimeDelayCounter = 5 * 30;//抓取超时判定，因为抓硬的东西，手爪有可能闭合不到位
                nStep = STEP_GRAB;
                ros::spinOnce();
                sleep(1);
                continue;
            }
        }

        // 5、抓取物品
        if(nStep == STEP_GRAB)
        {
            VelCmd(0,0,0);
            nTimeDelayCounter --;
            if(nTimeDelayCounter % 30 == 0)
                ROS_INFO("[wpr2_grab_bottle] 抓取倒计时 ... %d",nTimeDelayCounter / 30);
            if( joints_arrived == true || nTimeDelayCounter <= 0)
            {
                ROS_WARN("[wpr2_grab_bottle] 抓取完成，往上抬！");
                
                right_position[0] = 1.6673;
                right_position[1] = 1.5699;
                right_position[2] = -1.5818;
                right_position[3] = 1.3601;
                right_position[4] = 1.6793;
                right_position[5] = -1.3631;
                RightArmAction();

                nTimeDelayCounter = 5 * 30;
                nStep = STEP_OBJ_UP; 
                continue;     
            }
        }

        // 6、将物品上抬
        if(nStep == STEP_OBJ_UP)
        {
            nTimeDelayCounter --;
            if(nTimeDelayCounter % 30 == 0)
                ROS_INFO("[wpr2_grab_bottle] 抬起倒计时 ... %d",nTimeDelayCounter / 30);
            if( joints_arrived == true || nTimeDelayCounter <= 0)
            {
                ROS_WARN("[wpr2_grab_bottle] 上抬完毕，开始往后退！");

                chest_height_msg.position[0] = 0.7;
                ROS_WARN("[wpr2_grab_bottle] 升降高度 = %.2f" ,chest_height_msg.position[0]);
                chest_height_pub.publish(chest_height_msg);

                ctrl_msg.data = "odom_delta reset";
                odom_ctrl_pub.publish(ctrl_msg);

                fMoveTargetX = -grab_forward_range;
                fMoveTargetY = 0;

                nStep = STEP_BACKWARD;
                continue;    
            }
        }
       
        // 7、带着物品后退
        if(nStep == STEP_BACKWARD)
        {
            float vx,vy;
            vx = (fMoveTargetX - odom_delta.x)/2;
            vy = (fMoveTargetY - odom_delta.y)/2;

            VelCmd(vx,vy,0);
            ROS_INFO("[后退] 目标(%.2f %.2f)  里程(%.2f , %.2f) 速度(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, odom_delta.x ,odom_delta.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                ctrl_msg.data = "odom_delta reset";
                odom_ctrl_pub.publish(ctrl_msg);
                nTimeDelayCounter = 0;

                ROS_WARN("[wpr2_grab_bottle] 后退完毕，开始折叠手臂持有物品！");
                // 持有物品的姿势
                right_position[0] = -0.1593;
                right_position[1] = 1.5128;
                right_position[2] = -0.0382;
                right_position[3] = 1.7225;
                right_position[4] = 0.0637;
                right_position[5] = 1.5130;
                RightArmAction();

                nTimeDelayCounter = 5 * 30;
                nStep = STEP_HOLDING;
                continue;
            }
        }

        // 8、持有物品
        if(nStep == STEP_HOLDING)
        {
            nTimeDelayCounter --;
            if(nTimeDelayCounter % 30 == 0)
                ROS_INFO("[wpr2_grab_bottle] 收臂倒计时 ... %d",nTimeDelayCounter / 30);
            if( joints_arrived == true  || nTimeDelayCounter <= 0)
            {
                ROS_WARN("[wpr2_grab_bottle] 抓取动作完成！");

                nTimeDelayCounter = 0;
                nStep = STEP_DONE;
                continue;    
            }
        }

        // 9、抓取任务完毕
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