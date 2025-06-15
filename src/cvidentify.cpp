#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

void rgbcallback(const sensor_msgs::ImageConstPtr& msg)
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

    // Display the image in a window
    imshow("RGB ", img);
    waitKey(1);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "cvidentify_node");
    ros::NodeHandle nh;

    ros::Subscriber rgb_sub = nh.subscribe("/camera/image_raw", 1, rgbcallback);

    namedWindow("RGB ");
    ros::spin();
}

