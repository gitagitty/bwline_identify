# 黑线识别

## 一、消息格式

传输给串口的消息格式为

```
frame[0] = 0x0A;  // 帧头

frame[1] = uint8_t;  // 左x值

frame[2] = uint8_t;  // 右x值

frame[3] = 0x0D;  // 帧尾
```

1. 关于x的解释
   计算的是每个像素点距离相机中心的距离的平均值（厘米）
   左x值为左半边画面的识别线到中心的平均距离
   右x类似



## 二、launch文件说明

```
  <node name="ycrcb_node" pkg="bwline_id" type="ycrcb">
    <param name="low_y" value="0"/>
    <param name="high_y" value="80"/>
    <param name="low_cr" value="100"/>
    <param name="high_cr" value="150"/>
    <param name="low_cb" value="100"/>
    <param name="high_cb" value="150"/>
    <param name="start_value" value="0.0"/>
    <param name="end_value" value="0.7"/>
    <param name="input_topic" value="/camera/color/image_raw"/>
    <param name="depth_topic" value="/camera/depth/image_rect_raw"/>
  </node>


      <!-- 启动串口通信 -->
  <node name="serial_port" pkg="bwline_id" type="serial_port" >
    <param name="port" value="/dev/ttyUSB0"/>
  </node>
```

除了以上代码，其他launch文件中的部分为相机启动代码

```
<param name="low_y" value="0"/>
<param name="high_y" value="80"/>
<param name="low_cr" value="100"/>
<param name="high_cr" value="150"/>
<param name="low_cb" value="100"/>
<param name="high_cb" value="150"/>
```

此部分控制Y，Cr，Cb的值

```
<param name="start_value" value="0.0"/>
<param name="end_value" value="0.7"/>
```

此部分为人工设置截取画面部分，0.0为画面最上方，1.0为画面最下方
从start开始，到end结束截取画面

```
<param name="input_topic" value="/camera/color/image_raw"/>
<param name="depth_topic" value="/camera/depth/image_rect_raw"/>
```
此部分控制rgb和深度图像话题

```
<param name="port" value="/dev/ttyUSB0"/>
```

这个值为小电脑连接串口的名字，通常不用变更
