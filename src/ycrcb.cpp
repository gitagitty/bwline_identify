#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/utils.h" // 包含自定义的工具函数
#include <bwline_id/Results.h>
#include <image_transport/image_transport.h>
#include <utility>
#include <geometry_msgs/Point.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>
#include <chrono>



using namespace cv;
using namespace std;


// YCrCb默认阈值范围
int iLowY = 0, iHighY = 100;
int iLowCr = 120, iHighCr = 140;
int iLowCb = 120, iHighCb = 140;
int iLowH = 0; // HSV默认阈值范围
int iHighH = 180;
int iLowS = 0; // HSV默认阈值范围
int iHighS = 115;
int iLowV = 0; // HSV默认阈值范围
int iHighV = 88;  
int iLowR = 0; // RGB默认阈值范围
int iHighR = 90;
int iLowG = 0; // RGB默认阈值范围
int iHighG = 90;
int iLowB = 0; // RGB默认阈值范围
int iHighB = 90;

string rgbImageTopic_ = "/camera/color/image_raw"; // 输入图像话题
string depthImageTopic_ = "/camera/depth/image_rect_raw"; // 深度图像话题

bwline_id::Results results;
float start_value = 0.0; // 开始截取画面比例，0为最上端，1为最下端
float end_value = 0.7; // 结束截取画面比例，0为最上端，1为最下端


void imageCallback(const sensor_msgs::ImageConstPtr& rbg_msg,
                   const sensor_msgs::ImageConstPtr& depth_msg)
{
    try
    {
        Mat input_image = cv_bridge::toCvShare(rbg_msg, sensor_msgs::image_encodings::BGR8)->image;
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        Mat depth_img = cv_ptr->image;
        // 转换为YCrCb颜色空间
        Mat ycrcb_image;
        cvtColor(input_image, ycrcb_image, COLOR_BGR2YCrCb);

        Mat hsv_image;
        cvtColor(input_image, hsv_image, COLOR_BGR2HSV);
        hsv_image = clahe(hsv_image, 2); // 对V通道进行直方图均衡化
        hsv_image = threshold(hsv_image, iLowH,iHighH,iLowS,iHighS,iLowV,iHighV); // 对HSV图像进行阈值处理

        // Mat equalized_image = equalize(ycrcb_image, 0); // 对Y通道进行直方图均衡化
        // 或者使用CLAHE
        ycrcb_image = clahe(ycrcb_image, 0); // 对Y通道进行CLAHE处理


        // 阈值处理
        Mat thresholded_yrbimg = threshold(ycrcb_image, iLowY, iHighY, iLowCr, iHighCr, iLowCb, iHighCb);
        // Mat thresholded_rgbimg = threshold( input_image, iLowR, iHighR, iLowG, iHighG, iLowB, iHighB);

        // Mat thresholded_img = thresholded_yrbimg & thresholded_rgbimg & hsv_image; // 将阈值处理后的图像与原图进行按位与操作
        // Canny边缘检测
        Mat canny_img = canny_convert(thresholded_yrbimg);

        // 计算结果
        results = calculate(canny_img, depth_img, start_value, end_value); // 调整fraction参数以修改斜率范围

        imshow("input_image", input_image);
        // imshow("hsv_image", hsv_image);
        // imshow("thresholded_yrbimg", thresholded_yrbimg);
        // imshow("thresholded_rgbimg", thresholded_rgbimg);
        imshow("thresholded_img", thresholded_yrbimg);
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
    nh.getParam("ycrcb_node/low_y", iLowY);
    nh.getParam("ycrcb_node/high_y", iHighY);
    nh.getParam("ycrcb_node/low_cr", iLowCr);
    nh.getParam("ycrcb_node/high_cr", iHighCr);
    nh.getParam("ycrcb_node/low_cb", iLowCb);
    nh.getParam("ycrcb_node/high_cb", iHighCb);
    nh.getParam("ycrcb_node/input_topic", rgbImageTopic_);
    nh.getParam("ycrcb_node/depth_topic", depthImageTopic_);
    nh.getParam("ycrcb_node/start_value",start_value);//开始截取画面比例，0为最上端，1为最下端
    nh.getParam("ycrcb_node/end_value",end_value);//结束截取画面比例，0为最上端，1为最下端
    std::chrono::high_resolution_clock::time_point start_time_ = std::chrono::high_resolution_clock::now();
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> slamSyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image>* rimg_sub_ ;             // rgb图像输入
    message_filters::Subscriber<sensor_msgs::Image>* dimg_sub_;              // 深度图像输入
    message_filters::Synchronizer<slamSyncPolicy>* sync_;
    rimg_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, rgbImageTopic_, 1);
    dimg_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh, depthImageTopic_, 1);

    sync_ = new  message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(10), *rimg_sub_, *dimg_sub_);
    sync_->registerCallback(bind(imageCallback, _1, _2));
    ros::Publisher results_pub = nh.advertise<bwline_id::Results>("results", 10);
    ros::Rate loop_rate(30);

    namedWindow("YCrCb Thresholds", WINDOW_AUTOSIZE);
    createTrackbar("Low Y", "YCrCb Thresholds", &iLowY, 255);
    createTrackbar("High Y", "YCrCb Thresholds", &iHighY, 255);
    createTrackbar("Low Cr", "YCrCb Thresholds", &iLowCr, 255);
    createTrackbar("High Cr", "YCrCb Thresholds", &iHighCr, 255);
    createTrackbar("Low Cb", "YCrCb Thresholds", &iLowCb, 255);
    createTrackbar("High Cb", "YCrCb Thresholds", &iHighCb, 255);

    /* namedWindow("HSV Thresholds", WINDOW_AUTOSIZE);
    createTrackbar("Low H", "HSV Thresholds", &iLowH, 180);
    createTrackbar("High H", "HSV Thresholds", &iHighH, 180);
    createTrackbar("Low S", "HSV Thresholds", &iLowS, 255);
    createTrackbar("High S", "HSV Thresholds", &iHighS, 255);
    createTrackbar("Low V", "HSV Thresholds", &iLowV, 255);
    createTrackbar("High V", "HSV Thresholds", &iHighV, 255);

    namedWindow("RGB Thresholds", WINDOW_AUTOSIZE);
    createTrackbar("Low R", "RGB Thresholds", &iLowR, 255);
    createTrackbar("High R", "RGB Thresholds", &iHighR, 255);
    createTrackbar("Low G", "RGB Thresholds", &iLowG, 255);
    createTrackbar("High G", "RGB Thresholds", &iHighG, 255);
    createTrackbar("Low B", "RGB Thresholds", &iLowB, 255);
    createTrackbar("High B", "RGB Thresholds", &iHighB, 255); */
    while (ros::ok())
    {
        // 发布结果
        results_pub.publish(results);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}