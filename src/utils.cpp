#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../include/utils.h" 
#include <bwline_id/Results.h>

using namespace cv;
using namespace std;

const double K[9] = {609.6251831054688, 0.0, 317.4799499511719, 0.0, 608.375, 238.7157745361328, 0.0, 0.0, 1.0};



Mat equalize(Mat &input_image, int channel)
{
    Mat output_image;
    vector<Mat> channels;
    split(input_image, channels);
    
    if (channel < 0 || channel >= channels.size()) {
        ROS_ERROR("Invalid channel index: %d", channel);
        return input_image; // Return original image if channel is invalid
    }
    
    equalizeHist(channels[channel], channels[channel]);
    merge(channels, output_image);
    
    return output_image;
}

Mat clahe(Mat &input_image, int channel)
{
    Mat output_image;
    vector<Mat> channels;
    split(input_image, channels);
    
    if (channel < 0 || channel >= channels.size()) {
        ROS_ERROR("Invalid channel index: %d", channel);
        return input_image; // Return original image if channel is invalid
    }
    
    Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8)); // Create CLAHE object with clip limit and tile grid size
    clahe->apply(channels[channel], channels[channel]);
    
    merge(channels, output_image);
    
    return output_image;
}

Mat threshold(Mat &input_image, int low1, int high1, int low2, int high2,
                        int low3, int high3)
{
    Mat output_image;
    inRange(input_image, Scalar(low1, low2, low3),
                Scalar(high1, high2, high3), output_image);
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(output_image, output_image, MORPH_OPEN, element);
    morphologyEx(output_image, output_image, MORPH_CLOSE, element);
    return output_image;
}
Mat canny_convert(Mat &input_image)
{
    Mat canny_image;
    Canny(input_image, canny_image, 100, 200);
    return canny_image;
}

Mat rotate(Mat &input_image, double angle)
{
    Point2f center(input_image.cols / 2.0, input_image.rows / 2.0);
    Mat rotation_matrix = getRotationMatrix2D(center, angle, 1.0);
    Mat rotated_image;
    warpAffine(input_image, rotated_image, rotation_matrix, input_image.size());
    return rotated_image;
}

bwline_id::Results calculate(cv::Mat &input_image, cv::Mat &depth_img ,float start_value, float end_value)
{
    // 遍历thresholded_img中的每个像素
    double nTargetXL = 0.0;
    double nTargetXR = 0.0;
    int nImgWidth = input_image.cols;
    int nImgHeight = input_image.rows;
    int nImgChannels = input_image.channels();
    int PixCountL = 0; // 左半边像素计数
    int PixCountR = 0; // 右半边像素计数
    bwline_id::Results results;
    results.p_xl = 50; // 初始化左半边目标颜色的平均X坐标
    results.p_xr = 50; // 初始化右半边目标颜色的
    PixelData pixel_data;

    
    for (int y = (int)(start_value * nImgHeight); y < (int)(end_value * nImgHeight); y++)
    {
        for (int x = 0; x < nImgWidth; x++)
        {
            // 获取当前像素的值
            if (input_image.data[y*nImgWidth + x] == 255) // 如果像素值大于0，表示该像素是目标颜色
            {
                if (x<= nImgWidth / 2) // 如果像素在左半边
                {
                    pixel_data = findx(x, y, depth_img,PixCountL);// 累加左半边的X坐标
                    nTargetXL -= pixel_data.x; // 累加左半边的X坐标
                    PixCountL = pixel_data.count; // 更新左半边像素计数
                }
                else // 如果像素在右半边
                {
                    pixel_data = findx(x, y, depth_img,PixCountR); // 累加右半边的X坐标
                    nTargetXR += pixel_data.x; // 累加右半边的X坐标
                    PixCountR = pixel_data.count; // 更新右半边像素计数
                }
            }
        }
    }
    // ROS_WARN("p_xl = %d, p_xr = %d \n",nTargetXL, nTargetXR);

    if(PixCountL >= 300) {
    nTargetXL /= PixCountL;
    results.p_xl = max(0, min(200, static_cast<int>(nTargetXL)));
} else {
    results.p_xl = 255;
}

if(PixCountR >= 300) {
    nTargetXR /= PixCountR;
    results.p_xr = max(0, min(200, static_cast<int>(nTargetXR)));
} else {
    results.p_xr = 255;
}

    ROS_WARN("p_xl = %d, p_xr = %d \n",results.p_xl, results.p_xr);
    // ROS_WARN("PixCountL = %d, PixCountR = %d \n",PixCountL, PixCountR);
    
    return results;

}

PixelData findx(int pixel_x, int pixel_y, cv::Mat &depth_img,int counter)
{
    // ROS_WARN("width = %d, height = %d", depth_img.cols, depth_img.rows);
    try
        {
            PixelData pixel_data;
            if (pixel_x >= depth_img.cols || pixel_y >= depth_img.rows)
            {
                ROS_WARN("Pixel coordinates out of bounds.");
                pixel_data.x = 0.0;
                pixel_data.count = counter;
                return pixel_data; // 返回0表示未找到有效深度值
            }

            

            uint16_t depth_value = depth_img.at<uint16_t>(pixel_y, pixel_x); // 深度值是 16 位无符号整数（单位：毫米）

            if(depth_value !=0)
            {
                double depth_m = depth_value / 10.0; // 转换为cm

                double fx = K[0];
                double cx = K[2];
                double X = (pixel_x - cx) * depth_m / fx;
                // ROS_WARN("X = %f", X);
                counter ++;
                pixel_data.x = X; // 计算得到的X坐标
                pixel_data.count = counter; // 更新计数  
                return pixel_data; // 返回计算得到的X坐标
            }
        }
        catch (const cv::Exception& e)
        {
            ROS_ERROR("OpenCV exception: %s", e.what());
            PixelData pixel_data;
            pixel_data.x = 0.0;
            pixel_data.count = counter;
            return pixel_data; // 返回0表示未找到有效深度值
        }
}

