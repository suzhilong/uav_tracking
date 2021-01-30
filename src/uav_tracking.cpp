/*
use to control uav fly to track target
*/

#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>
#include <Eigen/Core>
#include <vector>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

using namespace std;

ros::Publisher trajectory_pub;
geometry_msgs::PointStamped current_position;

float linear_smoothing_navigation_step = 1;
int flag_tasks_OK = 0;
vector<int> Mid(375,240);//图像中心点

void flyToTarget(const std_msgs::UInt16MultiArray&);
void updateUavPosition(const geometry_msgs::PointStamped&);
double getDistanceToTarget(const Eigen::Vector3d&);
bool reachTargetPosition(const Eigen::Vector3d&, float);
bool linearSmoothingNavigationTask(const Eigen::Vector3d&);
vector<int> getBoxMid(int, int, int, int);
Eigen::Vector3d getTarget(const vector<int>&);


int main(int argc, char** argv) {
    ROS_INFO("uav tracking is running...");
    ros::init(argc, argv, "UAV Tracking");
    ros::NodeHandle nh;
    // Create a private node handle for accessing node parameters.
    ros::NodeHandle nh_private("~");

    std::string uav_name = "";
    uav_name = "firefly";
    ros::param::get("~mav_name",uav_name);
    string topic_dest = "/uav_tracking/target_point";

    // 订阅话题
    // /odometry_sensor1/position   无人机位置信息(包含噪声)
    ros::Subscriber position_sub = nh.subscribe(std::string("/"+uav_name+"/odometry_sensor1/position").c_str(), 10, &updateUavPosition);
    ros::Subscriber destination_sub = nh.subscribe(topic_dest, 10, &flyToTarget);

    trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);


    ros::Rate loop_rate(1);
    while (ros::ok())
    { 
        // ROS_INFO("UAV navigation is running...");

        ros::spinOnce();
        loop_rate.sleep();
    }
}


/*
得到中心点
*/
vector<int> getBoxMid(int x, int y, int w, int h)
{
    vector<int> boxMid(0, 0);
    boxMid[0] = x + w/2;
    boxMid[1] = y + h/2;
    return boxMid;
}


/*
得到目标点：
通过比较 boxMid 和 Mid，得到应该往current_position的哪边走
*/
Eigen::Vector3d getTarget(const vector<int>& boxMid)
{
    Eigen::Vector3d target3d(0,0,0);
    return target3d;
}


/*
Description: 
    void flyToTarget(const std_msgs::UInt16MultiArray& targetBox)
    飞到目标点

Parameters:
    targetBox 目标在图片中的位置信息
    ｘ,y,width,height

Return:
    无
*/
void flyToTarget(const std_msgs::UInt16MultiArray& targetBox)
{
    //把target转化为飞行目标点
    vector<int> boxMid = getBoxMid(targetBox.data[0],targetBox.data[1],targetBox.data[2],targetBox.data[3]);

    Eigen::Vector3d target = getTarget(boxMid);

    bool ok = linearSmoothingNavigationTask(target);
    if(!ok)
        cout << "fail to fly to target..." << endl;
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

    if (reachTargetPosition(target,0.2))
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