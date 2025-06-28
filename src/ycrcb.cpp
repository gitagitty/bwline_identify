#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Int32.h>





using namespace cv;
using namespace std;
//黑是0-60，白是200-255
static int iLowY = 0;
static int iHighY = 85;
static int iLowCr = 100;
static int iHighCr = 150;    
static int iLowCb = 100;
static int iHighCb = 150;
int p_x = 0; // 用于存储目标颜色的中心点X坐标
int nPixCount = 0;

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
    Mat ycrcb_img;
    cvtColor(img, ycrcb_img, COLOR_BGR2YCrCb);

    // Equalize the Y channel to improve contrast
    vector<Mat> ycrcb_channels;
    split(ycrcb_img, ycrcb_channels);
    //equalizeHist(ycrcb_channels[0], ycrcb_channels[0]); // 均衡化 Y 通道
    Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8)); // Create CLAHE object with clip limit and tile size
    clahe->apply(ycrcb_channels[0], ycrcb_channels[0]); // Apply CLAHE to the Y channel
    merge(ycrcb_channels, ycrcb_img);



    // Threshold the YCrCb image to get only desired colors
    Mat thresholded_img;
    inRange(ycrcb_img, Scalar(iLowY, iLowCr, iLowCb), Scalar(iHighY, iHighCr, iHighCb), thresholded_img);

    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(thresholded_img, thresholded_img, MORPH_OPEN, element);
    morphologyEx(thresholded_img, thresholded_img, MORPH_CLOSE, element);

    // Perform Canny edge detection on the thresholded image
    Mat canny_img;
    cv::Canny(thresholded_img, canny_img, 50, 150); // 使用Canny边缘检测

    // 遍历thresholded_img中的每个像素
    int nTargetXL = 0;
    int nTargetXR = 0;
    int nTargetX = 0; // 目标颜色的平均X坐标
    int nTargetY = 0;
    int nImgWidth = canny_img.cols;
    int nImgHeight = canny_img.rows;
    int nImgChannels = canny_img.channels();
    nPixCount = 0;
    int PixCountL = 0; // 左半边像素计数
    int PixCountR = 0; // 右半边像素计数

    /* y = 0.4height, x = 2x,y = height ,x = 0.5 x, (width - x)* (-2.5 y/height +3) = width - v,v = width + (x - width)  */
    
    for (int y = 0.4 * nImgHeight; y < nImgHeight; y++)
    {
        for (int x = 0; x < nImgWidth; x++)
        {
            // 获取当前像素的值
            if (canny_img.data[y*nImgWidth + x] == 255) // 如果像素值大于0，表示该像素是目标颜色
            {
                if (x<= nImgWidth / 2) // 如果像素在左半边
                {
                    nTargetXL += nImgWidth / 2 + (x - nImgWidth / 2) * (-2.5*y / nImgHeight +3 ); // 累加左半边的X坐标
                    PixCountL++; // 增加左半边像素计数
                }
                else // 如果像素在右半边
                {
                    nTargetXR += nImgWidth / 2 + (x - nImgWidth / 2) * (-2.5*y / nImgHeight +3); // 累加右半边的X坐标
                    PixCountR++; // 增加右半边像素计数
                }
                nTargetY += y;
                nPixCount++;
            }
        }
    }

    if(nPixCount > 0 && PixCountL > 0 && PixCountR > 0) // 如果找到了目标颜色的像素
    {
        nTargetXL /= PixCountL; // 计算目标颜色的平均X坐标
        nTargetXR /= PixCountR; // 计算目标颜色的平均X坐标
        nTargetX = (nTargetXL + nTargetXR) * 0.5; // 计算目标颜色的平均X坐标

        nTargetY /= nPixCount; // 计算目标颜色的平均Y坐标
        
        p_x = nTargetX*254/ nImgWidth;// 将像素坐标转换为0-254范围的值
        ROS_INFO("Target Center: (%d, %d), left x: %d, right x : %d, publish_x = %d ,Imgheight = %d, Imgwidth = %d, Pixcount = %d, PixcountL = %d, PixcountR = %d \n", 
            nTargetX, nTargetY, nTargetXL,nTargetXR, p_x, nImgHeight,nImgWidth, nPixCount, PixCountL, PixCountR);
    }
    
    // Display the images
    imshow("rgb", img);
    imshow("result", thresholded_img);
    imshow("canny", canny_img);
    
    waitKey(50);
}









int main(int argc, char** argv)
{
    ros::init(argc, argv, "ycrcb_node");
    ros::NodeHandle nh;

    ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw", 1, hsvcallback);
    ros::Rate loop_rate(30);


    ros::Publisher centre_pub = nh.advertise<std_msgs::Int32>("/centre_value", 10);


    namedWindow("YCrCb Thresholds", WINDOW_AUTOSIZE);
    createTrackbar("Low Y", "YCrCb Thresholds", &iLowY, 255);
    createTrackbar("High Y", "YCrCb Thresholds", &iHighY, 255);
    createTrackbar("Low Cr", "YCrCb Thresholds", &iLowCr, 255);
    createTrackbar("High Cr", "YCrCb Thresholds", &iHighCr, 255);
    createTrackbar("Low Cb", "YCrCb Thresholds", &iLowCb, 255);
    createTrackbar("High Cb", "YCrCb Thresholds", &iHighCb, 255);


    while(ros::ok())
    {
        std_msgs::Int32 centre_msg;
        centre_msg.data = p_x;
        // 发布目标颜色的中心点X坐标
        centre_pub.publish(centre_msg);
        ros::spinOnce(); // 处理回调函数
        loop_rate.sleep(); // 控制循环频率
    }

    return 0;
}