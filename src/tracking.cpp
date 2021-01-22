#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>
#include <Eigen/Core>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "pch.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
 
using namespace cv;
using namespace std;

ros::Subscriber position_sub;   // /odometry_sensor1/position   无人机位置信息(包含噪声)
ros::Subscriber video_left_sub;  // /vi_sensor/left/image_raw
ros::Subscriber video_right_sub; // /vi_sensor/right/image_raw
ros::Publisher trajectory_pub;
ros::Publisher target_pub;
geometry_msgs::PointStamped current_position;

float MAX_ERROR = 0.3; //与目标点的最大误差距离，因为数据有误差，所以需要根据运动尺度设置合适的值
float HEIGHT = 2.0; //飞机起飞高度
float linear_smoothing_navigation_step = 0.5; //数字越大导航速度越快
bool flag_gps_initialized_OK = false;
int flag_tasks_OK = 0;
Eigen::Vector3d home(0.0, 0.0, 0.0);
std::pair<int, int> targetPoint({0, 0});
string trackerType = "KCF"; //可选择"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "MOSSE", "CSRT" 
cv::Ptr<Tracker> tracker;

void updateUavPosition(const geometry_msgs::PointStamped&);
double getDistanceToTarget(const Eigen::Vector3d&);
bool reachTargetPosition(const Eigen::Vector3d&, float);
bool linearSmoothingNavigationTask(const Eigen::Vector3d&);
void trackingTarget(const sensor_msgs::Image&);


bool isFirstFrame = true;
cv::Rect2d bbox(0, 0, 0, 0);

int main(int argc, char** argv) {
    // ROS_INFO("video record is running...");
    ros::init(argc, argv, "Tracking");
    ros::NodeHandle nh;
    // Create a private node handle for accessing node parameters.
    ros::NodeHandle nh_private("~");

    std::string uav_name = "";  
    ros::param::get("~mav_name",uav_name);

    // 订阅话题
    position_sub = nh.subscribe(std::string("/"+uav_name+"/odometry_sensor1/position").c_str(), 10, &updateUavPosition);
    video_left_sub = nh.subscribe(std::string("/"+uav_name+"/vi_sensor/left/image_raw").c_str(), 10, &trackingTarget);
    // video_right_sub = nh.subscribe(std::string("/"+uav_name+"/vi_sensor/right/image_raw").c_str(), 10, &trackingTarget);

    // 发布话题 xxx_pub = nh.advertise<msg type>([topic], [Hz]);
    target_pub = nh.advertise<std::pair<int, int> >("/uav_tracking/target_point", 10);

    
    if (trackerType == "BOOSTING")
        tracker = TrackerBoosting::create();
    else if (trackerType == "MIL")
        tracker = TrackerMIL::create();
    else if (trackerType == "KCF")
        tracker = TrackerKCF::create();
    else if (trackerType == "TLD")
        tracker = TrackerTLD::create();
    else if (trackerType == "MEDIANFLOW")
        tracker = TrackerMedianFlow::create();
    else if (trackerType == "MOSSE")
        tracker = TrackerMOSSE::create();
    else (trackerType == "CSRT")
        tracker = TrackerCSRT::create();


    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        
        


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


/*
Description: 
    updateUavPosition(const geometry_msgs::PointStamped& msg)
    gps数据更新的回调函数.

Parameters:
    msg 位置信息

Return:
    无
*/
void updateUavPosition(const geometry_msgs::PointStamped& msg)
{
    if (!flag_gps_initialized_OK)
    {
        flag_gps_initialized_OK= true;
        home[0] = msg.point.x;
        home[1] = msg.point.y;
        home[2] = msg.point.z;
    }
    current_position = msg;
    // std::cout<<"UAV current position is: "<<msg.point.x<< msg.point.y<< msg.point.z<<std::endl;
}

/*
Description: 
    getDistanceToTarget(const Eigen::Vector3d& target)
    获取当前位置到指定位置位置的距离.

Parameters:
    target 需要飞达的位置点

Return:
    double 当前位置到达目标点的位置
*/
double getDistanceToTarget(const Eigen::Vector3d& target)
{
    // std::cout << std::to_string(current_position.point.x) << std::endl;
    // std::cout << std::to_string(current_position.point.y) << std::endl;
    // std::cout << std::to_string(current_position.point.z) << std::endl;
    double temp = 0;
    temp += pow((target[0] - current_position.point.x), 2);
    temp += pow((target[1] - current_position.point.y), 2);
    temp += pow((target[2] - current_position.point.z), 2);
    temp = sqrt(temp);
    return temp;
}

/*
Description: 
    reachTargetPosition(const Eigen::Vector3d& target, float max_error)
    判定是否到达指定的目标点.

Parameters:
    target 需要飞达的位置点
    max_error 允许的位置误差阈值,当前位置和目标位置小于该阈值时,判定无人机到达目标点

Return:
    bool 到达目标点时返回 true
            未到达目标点时返回 false
*/
bool reachTargetPosition(const Eigen::Vector3d& target, float max_error)
{
    double temp = getDistanceToTarget(target);
    // std::cout << "Dsitance: " << std::to_string(temp) << std::endl;
    // std::cout << "Max Error: " << std::to_string(max_error) << std::endl;
    if (temp < max_error)
        return true;
    return false;
}

/*
Description: 
    linearSmoothingNavigationTask(const Eigen::Vector3d& target)
    控制无人机从当前位置飞向指定位置.

Parameters:
    target 需要飞达的位置点

Return:
    bool 起飞结束后返回 true
            起飞过程中返回 false
*/
bool linearSmoothingNavigationTask(const Eigen::Vector3d& target)
{
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();

    if (reachTargetPosition(target, MAX_ERROR))
        return true;

    double dist = getDistanceToTarget(target);
    Eigen::Vector3d next_step;

    if(dist<linear_smoothing_navigation_step)
    {
        next_step = target;
    }
    else
    {
        next_step[0] = current_position.point.x+(target[0]-current_position.point.x)/dist*linear_smoothing_navigation_step;
        next_step[1] = current_position.point.y+(target[1]-current_position.point.y)/dist*linear_smoothing_navigation_step;
        next_step[2] = current_position.point.z+(target[2]-current_position.point.z)/dist*linear_smoothing_navigation_step;
    }

    double desired_yaw = 0.0; 

    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(next_step, desired_yaw, &trajectory_msg);
    trajectory_pub.publish(trajectory_msg);
    return false;
}

/*
Description: 
    trackingTarget(const sensor_msgs::Image& inputImg)
    追踪目标在图片中的位置

Parameters:
    inputImg 输入的图片

Return:
    无
*/
void trackingTarget(const sensor_msgs::Image& inputImg){
    cv::Mat curFrame = inputImg;
    
    if(isFirstFrame)
    {
        // 第一帧初始检测框的坐标
        bbox[0] = 0;
        bbox[1] = 0;
        bbox[2] = 100;
        bbox[3] = 100;

        // Display bounding box
        rectangle(curFrame, bbox, Scalar(255, 0, 0), 2, 1);
        imshow("Tracking", curFrame);

        //跟踪器初始化
        tracker->init(curFrame, bbox);

        isFirstFrame = false;
    }
    // Start timer 开始计时
    double timer = (double)getTickCount();
    // Update the tracking result 更新跟踪器算法
    bool ok = tracker->update(curFrame, bbox);
    // Calculate Frames per second (FPS) 计算FPS
    float fps = getTickFrequency() / ((double)getTickCount() - timer);

    if (ok)
    {
        // Tracking success : Draw the tracked object 如果跟踪到目标画框
        rectangle(curFrame, bbox, Scalar(255, 0, 0), 2, 1);
        //pub目标点
        
    }
    else
    {
        // Tracking failure detected. 
        
    }

    // Display tracker type on frame 展示检测算法类型
    putText(curFrame, trackerType + " Tracker", Point(100, 20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

    // Display FPS on frame 表示FPS
    putText(curFrame, "FPS : " + to_string(int(fps)), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

    target_pub.publish(targetPoint);
    return;
}


