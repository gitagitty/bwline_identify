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
int iLowY = 0, iHighY = 85;
int iLowCr = 100, iHighCr = 150;
int iLowCb = 100, iHighCb = 150;

string input_topic = "/camera/color/image_raw"; // 输入图像话题

bwline_id::Results results;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Mat input_image = cv_ptr->image;

        // 转换为YCrCb颜色空间
        Mat ycrcb_image;
        cvtColor(input_image, ycrcb_image, COLOR_BGR2YCrCb);

        Mat equalized_image = equalize(ycrcb_image, 0); // 对Y通道进行直方图均衡化

        // 阈值处理
        Mat thresholded_img = threshold(equalized_image, iLowY, iHighY, iLowCr, iHighCr, iLowCb, iHighCb);

        // Canny边缘检测
        Mat canny_img = canny_convert(thresholded_img);

        // 计算结果
        results = calculate(canny_img, 10); // 调整fraction参数以修改斜率范围

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
    ros::init(argc, argv, "ycrcb_node");
    ros::NodeHandle nh;
    nh.getParam("ycrcb/low_y", iLowY);
    nh.getParam("ycrcb/high_y", iHighY);
    nh.getParam("ycrcb/low_cr", iLowCr);
    nh.getParam("ycrcb/high_cr", iHighCr);
    nh.getParam("ycrcb/low_cb", iLowCb);
    nh.getParam("ycrcb/high_cb", iHighCb);
    nh.getParam("ycrcb/input_topic", input_topic);

    
    ros::Subscriber image_sub = nh.subscribe(input_topic, 1, imageCallback);
    ros::Publisher results_pub = nh.advertise<bwline_id::Results>("results", 10);
    ros::Rate loop_rate(30);

    namedWindow("YCrCb Thresholds", WINDOW_AUTOSIZE);
    createTrackbar("Low Y", "YCrCb Thresholds", &iLowY, 255);
    createTrackbar("High Y", "YCrCb Thresholds", &iHighY, 255);
    createTrackbar("Low Cr", "YCrCb Thresholds", &iLowCr, 255);
    createTrackbar("High Cr", "YCrCb Thresholds", &iHighCr, 255);
    createTrackbar("Low Cb", "YCrCb Thresholds", &iLowCb, 255);
    createTrackbar("High Cb", "YCrCb Thresholds", &iHighCb, 255);
    while (ros::ok())
    {
        // 发布结果
        results_pub.publish(results);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}