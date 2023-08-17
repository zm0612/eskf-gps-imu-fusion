# 误差状态卡尔曼滤波器(ESKF)融合IMU与GPS数据

|                   只使用IMU进行积分的结果                    |                     使用ESKF融合IMU和GPS                     |
| :----------------------------------------------------------: | :----------------------------------------------------------: |
| <img src="./data/images/0.png" alt="0" style="zoom: 33%;" /> | <img src="./data/images/1.png" alt="0" style="zoom: 33%;" /> |
| <img src="./data/images/3.png" alt="0" style="zoom: 33%;" /> | <img src="./data/images/2.png" alt="0" style="zoom: 33%;" /> |

实现方法请参考我的博客[《【附源码+代码注释】误差状态卡尔曼滤波(error-state Kalman Filter)实现GPS+IMU融合，EKF ErrorStateKalmanFilter GPS+IMU》](https://blog.csdn.net/u011341856/article/details/114262451)

## 1.  依赖库

Eigen

```shell
sudo apt-get install libeigen3-dev
```

Yaml

```shell
sudo apt-get install libyaml-cpp-dev
```

## 2. 编译

```shell
cd eskf-gps-imu-fusion
mkdir build
cd build
cmake ..
make 
```

## 3. 运行

```shell
cd eskf-gps-imu-fusion/build
./gps_imu_fusion /xxx/eskf-gps-imu-fusion/config/config.yaml /xxx/eskf-gps-imu-fusion/data
```

## 4.轨迹显示

执行完`./gps_imu_fusion`会生成轨迹文件

```shell
cd eskf-gps-imu-fusion/data
evo_traj tum fused.txt gt.txt gps_measurement.txt -p
```

> 需要安装evo，可以参考我的博客中的介绍：https://blog.csdn.net/u011341856/article/details/104594392?spm=1001.2014.3001.5501
