# 黑线识别

## 一、消息格式

传输给串口的消息格式为

```
frame[0] = 0x0A;  // 帧头

frame[1] = uint8_t;  // x值

frame[2] = uint8_t;  // slope值

frame[3] = 0x0D;  // 帧尾
```

1. 关于frame[1]的解释

其中frame[1]范围为0-255，代表两条线的加权平均位置，中心值为127.5
加权说明：给画面上方赋了更多权重，使得值的变化更平滑，符合实际情况

如果值大于127.5，说明两条线中心在画面偏右，说明机器狗在中心偏左

如果值小于127.5，说明两条线中心在画面偏左，说明机器狗在中心偏右

如果值等于0，说明画面中只有右半部分检测到线或者两条线中心在画面极左，说明机器狗在中心偏右

如果值等于255，说明画面中只有左半部分检测到线或者两条线中心在画面极右，说明机器狗在中心偏左

2. 关于frame[2]的解释

此值范围为0-255，代表两条线的平均斜率，中心值为127.5
其中参考系如下所示，原点在画面左上方，x正方向向右，y正方向向下
______________________________________________________________________________________x
|
|
|
|
|
|
|
|
y

程序以y为自变量，x为应变量计算画面两侧像素点拟合斜率

计算公式为
$$
127.5-k*(slope_l+slope_r)
$$

k为人工设置的控制斜率变化速率的参数，在launch文件中说明

如果值大于127.5，则两条斜线斜率和小于0，说明机器狗往左偏

如果值小于127.5，则两条斜线斜率和大于0，说明机器狗往右偏

## 二、launch文件说明

```
  <node name="ycrcb_node" pkg="bwline_id" type="ycrcb">
    <param name="low_y" value="0"/>
    <param name="high_y" value="85"/>
    <param name="low_cr" value="100"/>
    <param name="high_cr" value="150"/>
    <param name="low_cb" value="100"/>
    <param name="high_cb" value="150"/>
    <param name="k" value="10.0"/>
    <param name="input_topic" value="/camera/color/image_raw"/>
  </node>


      <!-- 启动串口通信 -->
  <node name="serial_port" pkg="bwline_id" type="serial_port" >
    <param name="port" value="/dev/ttyUSB0"/>
  </node>
```

除了以上代码，其他launch文件中的部分为相机启动代码

```
<param name="low_y" value="0"/>
    <param name="high_y" value="85"/>
    <param name="low_cr" value="100"/>
    <param name="high_cr" value="150"/>
    <param name="low_cb" value="100"/>
    <param name="high_cb" value="150"/>
```

此部分控制Y，Cr，Cb的值

```
 <param name="k" value="10.0"/>
```

此部分为人工设置的控制斜率变化速率的参数，k越大斜率变化范围越大，变化越快

```
<param name="port" value="/dev/ttyUSB0"/>
```

这个值为小电脑连接串口的名字，通常不用变更
