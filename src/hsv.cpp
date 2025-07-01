#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../include/utils.h" // 包含自定义的工具函数
#include <bwline_id/Results.h>


using namespace cv;
using namespace std;


// YCrCb默认阈值范围
int iLowH = 0, iHighH = 179;
int iLowS = 70, iHighS = 255;
int iLowV = 70, iHighV = 255;

string input_topic = "/camera/color/image_raw"; // 输入图像话题

bwline_id::Results results;

float k = 10.0; // 用于计算斜率的分数


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Mat input_image = cv_ptr->image;

        // 转换为HSV颜色空间
        Mat hsv_image;
        cvtColor(input_image, hsv_image, COLOR_BGR2HSV);

        // Mat equalized_image = equalize(hsv_image, 2); // 对Y通道进行直方图均衡化
        // 或者使用CLAHE
        Mat equalized_image = clahe(hsv_image, 2); // 对Y通道进行CLAHE处理

        // 阈值处理
        Mat thresholded_img = threshold(equalized_image, iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);

        // Canny边缘检测
        Mat canny_img = canny_convert(thresholded_img);

        // 计算结果
        results = calculate(canny_img, k); // 调整fraction参数以修改斜率范围

        imshow("input_image", input_image);
        imshow("thresholded_img", thresholded_img);
        imshow("canny_img", canny_img);
        waitKey(30); // 等待30毫秒以更新窗口
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "hsv_node");
    ros::NodeHandle nh;
    nh.getParam("hsv/low_h", iLowH);
    nh.getParam("hsv/high_h", iHighH);
    nh.getParam("hsv/low_s", iLowS);
    nh.getParam("hsv/high_s", iHighS);
    nh.getParam("hsv/low_v", iLowV);
    nh.getParam("hsv/high_v", iHighV);
    nh.getParam("hsv/k", k);
    nh.getParam("hsv/input_topic", input_topic);

    
    ros::Subscriber image_sub = nh.subscribe(input_topic, 1, imageCallback);
    ros::Publisher results_pub = nh.advertise<bwline_id::Results>("results", 10);
    ros::Rate loop_rate(30);

    namedWindow("HSV Thresholds", WINDOW_AUTOSIZE);
    createTrackbar("Low H", "HSV Thresholds", &iLowH, 179);
    createTrackbar("High H", "HSV Thresholds", &iHighH, 179);
    createTrackbar("Low S", "HSV Thresholds", &iLowS, 255);
    createTrackbar("High S", "HSV Thresholds", &iHighS, 255);
    createTrackbar("Low V", "HSV Thresholds", &iLowV, 255);
    createTrackbar("High V", "HSV Thresholds", &iHighV, 255);
    while (ros::ok())
    {
        // 发布结果
        results_pub.publish(results);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}