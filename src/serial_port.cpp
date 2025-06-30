#include "ros/ros.h"
#include <serial/serial.h>  // 串口头文件
#include <iostream>
#include <sstream>  // 用于数据转换
#include <bwline_id/Results.h>  // 自定义消息头文件

serial::Serial ser;
std::string port_;

// 串口初始化函数
void initSerialPort(const std::string &port, unsigned long baudrate) {
    try {
        // 打开串口
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        if (ser.isOpen()) {
            ROS_INFO("串口已打开：%s", port.c_str());
        } else {
            ROS_ERROR("无法打开串口：%s", port.c_str());
        }
    } catch (serial::IOException &e) {
        ROS_ERROR("串口打开失败: %s", e.what());
    }
}

// 发送数据到串口的函数
void sendToSerial(int x, int slope) {
    // 构造数据帧，帧头 0x0A，数据为 x, slope，帧尾 0x0D
    uint8_t frame[4];
    frame[0] = 0x0A;  // 帧头
    frame[1] = static_cast<uint8_t>(x);  // x值
    frame[2] = static_cast<uint8_t>(slope);  // slope值
    frame[3] = 0x0D;  // 帧尾

    // 发送数据帧
    try {
        ser.write(frame, sizeof(frame));
        ROS_INFO("数据已发送: x=%d", x);
    } catch (serial::IOException &e) {
        ROS_ERROR("发送数据失败: %s", e.what());
    }
}

// 接收并处理消息
void resultsCallback(const bwline_id::Results::ConstPtr& msg) {
        int x = msg->centre_x;
        int slope = msg->slope;

        // 打印坐标信息
        ROS_INFO("centre: %d , slope: %d", x, slope);
        // 发送坐标到串口
        sendToSerial(x, slope);
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle nh;
    nh.getParam("/serial_port/port", port_);

    // 初始化串口，假设串口路径为 /dev/ttyUSB0，波特率为 9600
    initSerialPort(port_, 9600);

    // 订阅ROS话题
    ros::Subscriber sub = nh.subscribe("results", 10, resultsCallback);

    ros::spin();  // 保持节点运行
    return 0;
}
