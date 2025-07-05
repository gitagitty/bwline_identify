#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <bwline_id/Results.h>
#include <chrono>


extern cv::Mat equalize(cv::Mat &input_image, int channel);
extern cv::Mat clahe(cv::Mat &input_image, int channel);
extern cv::Mat threshold(cv::Mat &input_image, int low1, int high1, int low2, int high2,
                        int low3, int high3);
extern cv::Mat canny_convert(cv::Mat &input_image);
extern cv::Mat rotate(cv::Mat &input_image, double angle);

extern bwline_id::Results calculate(cv::Mat &input_image,cv::Mat &depth_img ,float start_value, float end_value);
struct PixelData {
    double x;   // 存储计算得到的X坐标
    int count;  // 存储像素计数
};
extern PixelData findx(int pixel_x, int pixel_y, cv::Mat &depth_img, int counter);

extern const double K[9];


#endif // UTILS_H
