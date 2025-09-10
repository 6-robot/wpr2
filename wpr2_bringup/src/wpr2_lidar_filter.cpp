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

#include <math.h>	
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

#define BUFF_LEN 860

// C1雷达初始测距值会波动，先忽略一部分数值
static int ignore_count = 20;

static float front = 0.24;
static float back = -0.20;
static float side = 0.29;
static int nLidarPointNum = BUFF_LEN;

static std::string pub_topic;
class CWPCLidarFilter
{
public:
    CWPCLidarFilter();
private:
    ros::NodeHandle n;
    ros::Publisher scan_pub;
    ros::Subscriber scan_sub;
    int nLidarPointNum;
    double x_sin[BUFF_LEN];
    double y_cos[BUFF_LEN];
    float pnt_x[BUFF_LEN];
    float pnt_y[BUFF_LEN];
    void Resize(int inNum);
    void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

CWPCLidarFilter::CWPCLidarFilter()
{
    Resize(BUFF_LEN);
    scan_pub = n.advertise<sensor_msgs::LaserScan>(pub_topic,1);
    scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan_raw",1,&CWPCLidarFilter::LidarCallback,this);
}

void CWPCLidarFilter::Resize(int inNum)
{
    double kStep = (M_PI * 2) / inNum;
	for (int i = 0; i < inNum; i++)
	{
		x_sin[i] = sin(M_PI*1.0 - kStep*i);
		y_cos[i] = -cos(M_PI*1.0 - kStep*i);
	}
    nLidarPointNum = inNum;
}

void CWPCLidarFilter::LidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // ROS_INFO("[wpb_mani_lidar_filter]");
    if(ignore_count > 0)
    {
        ignore_count --;
        return;
    }
    int nRanges = scan->ranges.size();
    // ROS_WARN("[wpb_mani_lidar_filter] nRanges = %d",nRanges);
    if(nRanges != nLidarPointNum)
    {
        Resize(nRanges);
    }
    static sensor_msgs::LaserScan new_scan;
    new_scan.header.stamp = scan->header.stamp;
    new_scan.header.frame_id = scan->header.frame_id;
    new_scan.angle_max = scan->angle_max;
    new_scan.angle_min = scan->angle_min;
    new_scan.angle_increment = scan->angle_increment;
    new_scan.time_increment = scan->time_increment;
    new_scan.range_min = scan->range_min;
    new_scan.range_max = scan->range_max;
    new_scan.ranges.resize(nRanges);
    new_scan.intensities.resize(nRanges);
    for(int i=0 ; i<nRanges ; i++)
    {
        new_scan.ranges[i] = scan->ranges[i];
        pnt_x[i] = scan->ranges[i] * x_sin[i];
        pnt_y[i] = scan->ranges[i] * y_cos[i];
        if(pnt_x[i] > -side && pnt_x[i] < side && pnt_y[i] < front && pnt_y[i] > back)
        {
            new_scan.ranges[i] = new_scan.range_max+1.0;
        }
        new_scan.intensities[i] = scan->intensities[i];
    }
    scan_pub.publish(new_scan);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpr2_lidar_filter");
     ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("pub_topic", pub_topic, "/scan_filtered");

    CWPCLidarFilter lidar_filter;
    ros::spin();
}
