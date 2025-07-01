#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../include/utils.h" 
#include <bwline_id/Results.h>

using namespace cv;
using namespace std;

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
    Mat gray_image, canny_image;
    cvtColor(input_image, gray_image, COLOR_BGR2GRAY);
    Canny(gray_image, canny_image, 100, 200);
    return canny_image;
}

bwline_id::Results calculate(Mat &input_image, float fraction/* 修改斜率的范围，参数越大斜率范围越大 */)
{
    // 遍历thresholded_img中的每个像素
    int nTargetXL = 0;
    int nTargetXR = 0;
    int nTargetX = 0; // 目标颜色的平均X坐标
    int nTargetY = 0;
    int nTargetYL = 0; // 左半边目标颜色的平均Y坐标
    int nTargetYR = 0; // 右半边目标颜色的平均Y
    int nImgWidth = input_image.cols;
    int nImgHeight = input_image.rows;
    int nImgChannels = input_image.channels();
    int PixCountL = 0; // 左半边像素计数
    int PixCountR = 0; // 右半边像素计数
    int sumXL = 0; // 左半边像素的X坐标和
    int sumXR = 0; // 右半边像素的X坐标
    int sumXYL = 0; // 左半边像素的X坐标和Y坐标的乘积和
    int sumXYR = 0; // 右半边像素的X坐标和Y坐标的乘积和
    int sumY2L = 0; // 左半边像素的X坐标的平方和
    int sumY2R = 0; // 右半边像素的X
    float slope = 0; // 斜率
    bwline_id::Results results;

    
    for (int y = 0.4 * nImgHeight; y < nImgHeight; y++)
    {
        for (int x = 0; x < nImgWidth; x++)
        {
            // 获取当前像素的值
            if (input_image.data[y*nImgWidth + x] == 255) // 如果像素值大于0，表示该像素是目标颜色
            {
                if (x<= nImgWidth / 2) // 如果像素在左半边
                {
                    nTargetXL += nImgWidth / 2 + (x - nImgWidth / 2) * (-2.5*y / nImgHeight +3 ); // 累加左半边的X坐标
                    nTargetYL += y; // 累加左半边的Y坐标
                    sumXYL += x * y; // 累加左半边像素的X坐标和Y坐标的乘积
                    sumY2L += y * y; // 累加左半边像素的Y坐标的平方
                    sumXL += x; // 累加左半边像素的X坐标
                    PixCountL++; // 增加左半边像素计数
                }
                else // 如果像素在右半边
                {
                    nTargetXR += nImgWidth / 2 + (x - nImgWidth / 2) * (-2.5*y / nImgHeight +3); // 累加右半边的X坐标
                    nTargetYR += y; // 累加右半边的Y坐标
                    sumXYR += x * y; // 累加右半边像素的X坐标和Y坐标的乘积
                    sumY2R += y * y; // 累加右半边像素的Y坐标的平方
                    sumXR += x; // 累加右半边
                    PixCountR++; // 增加右半边像素计数
                }
            }
        }
    }

    if(PixCountL > 0 && PixCountL > 0) // 如果找到了目标颜色的像素
    {
        nTargetXL /= PixCountL; // 计算目标颜色的平均X坐标
        nTargetXR /= PixCountR; // 计算目标颜色的平均X坐标
        nTargetX = (nTargetXL + nTargetXR) * 0.5; // 计算目标颜色的平均X坐标

        nTargetY = (nTargetYL + nTargetYR) / (PixCountL + PixCountR); // 计算目标颜色的平均Y坐标
        
        results.centre_x = max(0, min(255,nTargetX*255/ nImgWidth));// 将像素坐标转换为0-254范围的值
        ROS_INFO("Target Center: (%d, %d), left x: %d, right x : %d, publish_x = %d ,Imgheight = %d, Imgwidth = %d, PixcountL = %d, PixcountR = %d \n", 
            nTargetX, nTargetY, nTargetXL,nTargetXR, results.centre_x, nImgHeight,nImgWidth, PixCountL, PixCountR);
    }

    if(PixCountL > 0 && PixCountR > 0) // 如果找到目标颜色的像素
    {
        float slopeL = (float)(PixCountL * sumXYL - sumXL * nTargetYL) / (PixCountL * sumY2L - nTargetYL * nTargetYL);
        float slopeR = (float)(PixCountR * sumXYR - sumXR * nTargetYR) / (PixCountR * sumY2R - nTargetYR * nTargetYR);
        slope = slopeL + slopeR; // 计算平均斜率
    }

    results.slope = min(255,max(0,(int)(-slope * fraction + 127.5))); // 根据fraction调整斜率范围
    
    return results;

}