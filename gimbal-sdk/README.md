# 阿木实验室G1吊舱SDK(Linx C++ )



## 介绍

- 阿木实验室G1吊舱是一款高性能低成本的光学吊舱
- 智能吊舱=云台+相机+AI芯片+人机交互软件+深度学习
- Linux SDK (GCC 7.5.0) 



## 下载SDK：

1. 克隆仓库 

   `git clone https://gitee.com/amovlab/gimbal-sdk.git`  
   
   

## 运行示例：

1、进入仓库使用如下命令进行SDK编译

```
mkdir build 
cd build
cmake ..
make 
```

2、可以在build文件夹中可以看到`GetGimbalState`、`GimbalSpeedControl`、`GimbalAngleAndSpeedControl`等可执行文件

3、接入串口，通过 `ls /dev/ttyUSB*`命令确保已经有串口 `/dev/ttyUSB*`

4、运行示例的方法:

- 获取吊舱的状态数据，包括IMU角度和编码器角度

```
./GetGimbalState -S /dev/ttyUSB0 -b 115200
```

- 控制吊舱的转动速度，下面的例子是以roll：10°/s 、pitch：10°/s、yaw：10°/s进行控制

```
 ./GimbalAngleRateControl -S /dev/ttyUSB0 -b 115200 -r 10 -p 10 -y 10
```

- 控制吊舱的转动角度并指定转动速度大小为10°/s，下面的例子转动到roll：10° 、pitch：10°、yaw：10°进行控制

```
 ./GimbalAngleControl -S /dev/ttyUSB0 -b 115200 -r 10 -p 10 -y 10 -rate 10
```

- 拍照和录像功能，图片和视频会保存至TF卡中

  ```
  ./CameraControl -a 1 #录像和停止录像  
  ./CameraControl -a 2 #拍照
  ```


## 联系我们

- 阿木实验室官网：https://www.amovlab.com/
- 阿木实验室论坛：https://bbs.amovlab.com/

