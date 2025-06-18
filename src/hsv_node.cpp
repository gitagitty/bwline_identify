#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Int32.h>


using namespace cv;
using namespace std;

static int iLowH = 0;
static int iHighH = 179;
static int iLowS = 70;
static int iHighS = 255;    
static int iLowV = 70;
static int iHighV = 255;
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
    Mat hsv_img;
    cvtColor(img, hsv_img, COLOR_BGR2HSV);

    // Equalize the V channel to improve contrast
/*     vector<Mat> hsv_channels;
    split(hsv_img, hsv_channels);
    equalizeHist(hsv_channels[2], hsv_channels[2]); // Equalize the V channel
    merge(hsv_channels, hsv_img); */

    //自适应直方图均衡化
    vector<Mat> hsv_channels;
    split(hsv_img, hsv_channels);// Equalize the V channel
    Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8)); // Create CLAHE object with clip limit and tile size
    clahe->apply(hsv_channels[2], hsv_channels[2]); // Apply CLAHE to the V channel
    merge(hsv_channels, hsv_img);



    // Threshold the HSV image to get only desired colors
    Mat thresholded_img;
    inRange(hsv_img, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresholded_img);
    

    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(thresholded_img, thresholded_img, MORPH_OPEN, element);
    morphologyEx(thresholded_img, thresholded_img, MORPH_CLOSE, element);


    // 遍历thresholded_img中的每个像素
    int nTargetX = 0;
    int nTargetY = 0;
    int nImgWidth = thresholded_img.cols;
    int nImgHeight = thresholded_img.rows;
    int nImgChannels = thresholded_img.channels();
    nPixCount = 0;
    for (int y = 300; y < nImgHeight; y++)
    {
        for (int x = 0; x < nImgWidth; x++)
        {
            // 获取当前像素的值
            if (thresholded_img.data[y*nImgWidth + x] == 255) // 如果像素值大于0，表示该像素是目标颜色
            {
                nTargetX += x;
                nTargetY += y;
                nPixCount++;
            }
        }
    }

    if(nPixCount > 0)
    {
        nTargetX /= nPixCount; // 计算目标颜色的平均X坐标
        nTargetY /= nPixCount; // 计算目标颜色的平均Y坐标
        
        p_x = nTargetX*254/639;// 将像素坐标转换为0-254范围的值
        // 在原图上绘制目标颜色的中心点
        circle(thresholded_img, Point(nTargetX, nTargetY), 5, Scalar(0, 0, 255), -1);
        printf("Target Center: (%d, %d), publish_x = %d Pixcount= %d\n", nTargetX, nTargetY, p_x, nPixCount);
    }
    
    // Display the images
    imshow("rgb", img);
    imshow("result", thresholded_img);
    
    waitKey(1);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "hsv_node");
    ros::NodeHandle nh;

    ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw", 1, hsvcallback);
    ros::Rate loop_rate(30);
    
    
    ros::Publisher centre_pub = nh.advertise<std_msgs::Int32>("/centre_value", 10);

    
    namedWindow("HSV Thresholds", WINDOW_AUTOSIZE);
    createTrackbar("Low H", "HSV Thresholds", &iLowH, 179);
    createTrackbar("High H", "HSV Thresholds", &iHighH, 179);
    createTrackbar("Low S", "HSV Thresholds", &iLowS, 255);
    createTrackbar("High S", "HSV Thresholds", &iHighS, 255);
    createTrackbar("Low V", "HSV Thresholds", &iLowV, 255);
    createTrackbar("High V", "HSV Thresholds", &iHighV, 255);
    
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