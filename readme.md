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

Glog
```shell
sudo apt-get install libgoogle-glog-dev
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
cd eskf-gps-imu-fusion
./build/gps_imu_fusion ./config/config.yaml ./data
```

## 4.轨迹显示

执行完`./gps_imu_fusion`会生成轨迹文件
```shell
cd eskf-gps-imu-fusion/data
python display_path.py
```

## 5.误差分析

__推荐使用工具__: [evo](https://github.com/MichaelGrupp/evo)
```shell
cd eskf-gps-imu-fusion/data
evo_traj tum fused.txt gt.txt gps_measurement.txt -p
```

## 6. 接入其他数据
如果需要接入其他数据，您需要将您的数据格式进行整理，以符合本算法的需求，参考`data/raw_data`文件夹中的数据格式，并且至少要在`accel-0.csv`,`gps-0.csv`,`gps_time.csv`,`gyro-0.csv`,`time.csv`文件中填入你的IMU和GPS数据。

**提示：**
- IMU的加速度和角速度需要使用前右下坐标系；
- GPS的数据按照：纬度、经度、高度填入，并且单位分别为度和米；
- 采集和生成自己的数据时，请尽量从静止和近水平面状态开始运动。

## 7. todo
- [ ] 增加初始化时重力对齐
- [ ] 增加初始化时bias估计
- [ ] ……
