#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <bwline_id/Results.h>


extern cv::Mat equalize(cv::Mat &input_image, int channel);
extern cv::Mat clahe(cv::Mat &input_image, int channel);
extern cv::Mat threshold(cv::Mat &input_image, int low1, int high1, int low2, int high2,
                        int low3, int high3);
extern cv::Mat canny_convert(cv::Mat &input_image);
extern cv::Mat rotate(cv::Mat &input_image, double angle);

extern bwline_id::Results calculate(cv::Mat &input_image, float fraction);

#endif // UTILS_H
