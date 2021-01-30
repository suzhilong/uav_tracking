#include <iostream>
#include <vector>
#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sensor_msgs/image_encodings.h> //包含对图像进行编码的函数
#include <cv_bridge/cv_bridge.h> //将ROS下的sensor_msgs/Image消息类型转化成cv::Mat。
#include<image_transport/image_transport.h> //image_transport 头文件用来在ROS系统中的话题上发布和订阅图象消息

//OpenCV2标准头文件 
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
 
using namespace cv;
using namespace std;

// ros::Subscriber video_left_sub;  // /vi_sensor/left/image_raw
// ros::Subscriber video_right_sub; // /vi_sensor/right/image_raw
ros::Publisher target_pub;//发布跟踪结果
// image_transport::Subscriber image_sub; //定义ROS图象接收器 ,订阅主题的变量
// image_transport::Publisher image_pub; //定义ROS图象发布器 ,发布主题的变量


void trackingTarget(cv::Mat);
void updateFrame(const sensor_msgs::ImageConstPtr&);

/*
Rect2d:
x; //左上角的x坐标
y; //左上角的y坐标
width; //矩形的宽度
height; //矩形的高度
*/
cv::Rect2d boundingBox(350, 200, 100, 80);
string trackerType = "KCF"; //可选择"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "MOSSE", "GOTURN" 
cv::Ptr<Tracker> tracker;
bool isFirstFrame = true;
bool DisplayTracking = false; //是否可视化跟踪，true的话可视化框容易卡死
bool save = false;//是否保存第一帧来观察选定目标

int main(int argc, char** argv) {
    if (trackerType == "BOOSTING"){
        tracker = cv::TrackerBoosting::create();
    }else if (trackerType == "MIL"){
        tracker = cv::TrackerMIL::create();
    }else if (trackerType == "KCF"){
        tracker = cv::TrackerKCF::create();
    }else if (trackerType == "TLD"){
        tracker = cv::TrackerTLD::create();
    }else if (trackerType == "MEDIANFLOW"){
        tracker = cv::TrackerMedianFlow::create();
    }else if (trackerType == "MOSSE"){
        tracker = cv::TrackerMOSSE::create();
    }else if(trackerType == "GOTURN"){
        tracker = cv::TrackerGOTURN::create();
    }else{
        ROS_INFO("wrong trackerType...");
    }
    cout << "-tracker type: " << trackerType << endl;
    // ROS_INFO("video record is running...");
    cout << "-main --------" << endl;
    ros::init(argc, argv, "Tracking");
    ros::NodeHandle nh;
    // Create a private node handle for accessing node parameters.
    ros::NodeHandle nh_private("~");

    std::string uav_name = "";
    uav_name = "firefly";
    ros::param::get("~mav_name",uav_name);
    string topic_img = "/" + uav_name+"/vi_sensor/left/image_raw";
    cout << "-topic: " << topic_img << endl;
    // 订阅话题
    // video_left_sub = nh.subscribe(std::string("/"+uav_name+"/vi_sensor/left/image_raw").c_str(), 10, &updateFrame);
    // video_right_sub = nh.subscribe(std::string("/"+uav_name+"/vi_sensor/right/image_raw").c_str(), 10, &trackingTarget);

    // 发布话题 xxx_pub = nh.advertise<msg type>([topic], [Hz]);
    target_pub = nh.advertise<std_msgs::UInt16MultiArray >("/uav_tracking/target_point", 1);

    //定义图象接受器，订阅话题是“~mav_name/vi_sensor/left/image_raw”
    cout << "-topic sub -------" << endl;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe(topic_img, 1, updateFrame);
    // image_pub = it.advertise("/image_converter/output_video", 1); //定义图象发布器


    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        cout << "ros::ok begin---------" << endl;


        ros::spinOnce();
        cout << "ros::ok end---------" << endl;
        loop_rate.sleep();
    }

    // ros::spin();
    // cout << "-again-------" << endl;
    return 0;
}


/*
Description: 
    updateFrame(const sensor_msgs::Image& inputImg)
    camera数据更新的回调函数. ROS和OpenCV的格式转换

Parameters:
    msgImg 图片msg

Return:
    无
*/
void updateFrame(const sensor_msgs::ImageConstPtr& msgImg)
{
    cout << "--updateFrame ----------" << endl;
    cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例 
    try
    { 
        cv_ptr = cv_bridge::toCvCopy(msgImg, sensor_msgs::image_encodings::RGB8); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针 
    } 
    catch(cv_bridge::Exception& e) //异常处理 
    { 
        ROS_ERROR("cv_bridge exception: %s", e.what()); 
        return; 
    } 

    trackingTarget(cv_ptr->image); //得到了cv::Mat类型的图象，在CvImage指针的image中，将结果传送给跟踪函数 
    
    
    cv::Mat frame = cv_ptr->image;
    cout << " row: " << frame.rows << " col:" << frame.cols << endl;
    // cv::imshow("Img", frame);
    if(save){
        //保存第一帧来选定目标
        cv::imwrite("./src/uav_tracking/firstFrame.jpg",frame);
        save = false;
    }
}


/*
Description: 
    trackingTarget()
    追踪目标在图片中的位置

Parameters:
    curFrame 当前帧

Return:
    无
*/
void trackingTarget(cv::Mat curFrame)
{
    cout << "---trackingTarget bagin -----------" << endl;
    if(isFirstFrame)
    {
        // 如果第一帧，初始化
        cout << "----initialization" << endl;
        // 1. 初始化检测框的坐标
        // boundingBox.x = 350;
        // boundingBox.y = 200;
        // boundingBox.width = 100;
        // boundingBox.height = 80;
        // 2. 手动框选
        boundingBox = selectROI(curFrame, false);

        if (DisplayTracking)
        {
            // Display bounding box
            cv::rectangle(curFrame, boundingBox, Scalar(255, 0, 0), 2, 1);
            cv::imshow("Tracking", curFrame);
        }

        //初始化跟踪器
        tracker->init(curFrame, boundingBox);

        isFirstFrame = false;
    }else{
        cout << "----already initialization" << endl;
    }

    // Start timer
    double timer = (double)getTickCount();
    // Update the tracking result
    // curFrame: current frame
    // boundingBox:	The bounding box that represent the new target location, 
    // if true was returned, not modified otherwise
    bool ok = tracker->update(curFrame, boundingBox);
    // Calculate Frames per second (FPS)
    float fps = getTickFrequency() / ((double)getTickCount() - timer);
    cout << "fps: " << fps << endl;

    if (DisplayTracking)
    {
        cout << "----displayTracking bagin -----------" << endl;
        if (ok)
        {
            // Tracking success : Draw the tracked object
            cv::rectangle(curFrame, boundingBox, Scalar(255, 0, 0), 2, 1);
        }
        else
		{
			// Tracking failure detected.
			cv::putText(curFrame, "Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
		}
        // Display tracker type on frame
        putText(curFrame, trackerType + " Tracker", Point(100, 20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
        // Display FPS on frame
        putText(curFrame, "FPS : " + to_string(int(fps)), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
        // Display frame.
		imshow("Tracking", curFrame);
        cout << "----displayTracking end -----------" << endl;
    }

    if (ok)
    {
        // Tracking success : Draw the tracked object
        cout << "-----tracking success" << endl;
    }
    else
    {
        cout << "-----tracking false" << endl;
    }

    // Tracking failure will use old bbox to update
    std_msgs::UInt16MultiArray targetBox;  //x,y,width,height

    targetBox.layout.dim.push_back(std_msgs::MultiArrayDimension());
    targetBox.layout.dim[0].size = 4;
    targetBox.layout.dim[0].stride = 1;
    targetBox.layout.dim[0].label = "box";

    targetBox.data.resize(4);
    targetBox.data[0] = boundingBox.x;
    targetBox.data[1] = boundingBox.y;
    targetBox.data[2] = boundingBox.width;
    targetBox.data[3] = boundingBox.height;

    cout << "x: " << targetBox.data[0] 
        << " y:" << targetBox.data[1]
        << " width: " << targetBox.data[2]
        << " height: " << targetBox.data[3] << endl;

    target_pub.publish(targetBox);
    cout << "---trackingTarget end -----------" << endl;
    return;
}