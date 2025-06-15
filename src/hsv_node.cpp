#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;

static int iLowH = 200;
static int iHighH = 250;
static int iLowS = 120;
static int iHighS = 135;    
static int iLowV = 120;
static int iHighV = 125;

void hsvcallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat img = cv_ptr->image;

    // Convert the image from BGR to HSV
    Mat hsv_img, ycrcb_img;
    cvtColor(img, hsv_img, COLOR_BGR2HSV);
    cvtColor(img, ycrcb_img, COLOR_BGR2YCrCb);

    // Equalize the V channel to improve contrast
/*     vector<Mat> hsv_channels;
    split(hsv_img, hsv_channels);
    equalizeHist(hsv_channels[2], hsv_channels[2]); // Equalize the V channel
    merge(hsv_channels, hsv_img); */

    //自适应直方图均衡化
    /* vector<Mat> hsv_channels;
    split(hsv_img, hsv_channels);// Equalize the V channel
    Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8)); // Create CLAHE object with clip limit and tile size
    clahe->apply(hsv_channels[2], hsv_channels[2]); // Apply CLAHE to the V channel
    merge(hsv_channels, hsv_img); */

    vector<Mat> ycrcb_channels;
    split(ycrcb_img, ycrcb_channels);
    //equalizeHist(ycrcb_channels[0], ycrcb_channels[0]); // 均衡化 Y 通道
    Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8)); // Create CLAHE object with clip limit and tile size
    clahe->apply(ycrcb_channels[0], ycrcb_channels[0]); // Apply CLAHE to the Y channel
    merge(ycrcb_channels, ycrcb_img);



    // Threshold the HSV image to get only desired colors
    Mat thresholded_img;
    //inRange(hsv_img, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresholded_img);
    inRange(ycrcb_img, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresholded_img);

    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(thresholded_img, thresholded_img, MORPH_OPEN, element);
    morphologyEx(thresholded_img, thresholded_img, MORPH_CLOSE, element);
    // Display the images
    imshow("rgb", img);
    //imshow("hsv", hsv_img);
    //imshow("ycrcb", ycrcb_img);
    imshow("result", thresholded_img);
    
    waitKey(1);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "hsv_node");
    ros::NodeHandle nh;

    ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw", 1, hsvcallback);

    namedWindow("HSV Thresholds", WINDOW_AUTOSIZE);
    createTrackbar("Low H", "HSV Thresholds", &iLowH, 255);
    createTrackbar("High H", "HSV Thresholds", &iHighH, 255);
    createTrackbar("Low S", "HSV Thresholds", &iLowS, 255);
    createTrackbar("High S", "HSV Thresholds", &iHighS, 255);
    createTrackbar("Low V", "HSV Thresholds", &iLowV, 255);
    createTrackbar("High V", "HSV Thresholds", &iHighV, 255);

    namedWindow("rgb");
    namedWindow("hsv");
    namedWindow("result");

    ros::Rate loop_rate(30);
    ros::spin();

    
    return 0;
}