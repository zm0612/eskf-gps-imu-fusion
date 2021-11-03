### 2021年9月17日更新：
有同学反应代码在编译和运行的过程中有一些bug，由于我最近工作有些忙，我先给出一个简单的办法补救一下，后续闲下来我再好好测试和修复，实在不好意思！

a. **编译报错：GeographicLib/LocalCartesian.hpp：没有那个文件或目录**

原因：我的cmakelists.txt中的文件路径设置错误了。

解决办法：最简单的解决办法就是安装Geographic库`sudo apt-get install libgeographic-dev`，然后重新编译问题就解决了。

b. **在Ubuntu20.04系统下可以成功编译，但是运行有问题**

解决办法：该问题并不是每一个Ubuntu20.04系统都会出现，看起来像是个别现象。我的开发环境是Ubuntu 18.04！

======================= 我只是一个分割线 ==============================

# ESKF融合IMU与GPS数据


![融合IMU数据之后的GPS轨迹效果](https://img-blog.csdnimg.cn/20210304150232490.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3UwMTEzNDE4NTY=,size_16,color_FFFFFF,t_70#pic_center)

绿色轨迹：ground truth
蓝色轨迹：fuse IMU and GPS
红色轨迹：GPS

实现方法请参考我的博客[《【附源码+代码注释】误差状态卡尔曼滤波(error-state Kalman Filter)实现GPS+IMU融合，EKF ESKF GPS+IMU》](https://blog.csdn.net/u011341856/article/details/114262451)

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
./gps_imu_fusion
```

## 4.轨迹显示

执行完`./gps_imu_fusion`会生成轨迹文件

```shell
cd eskf-gps-imu-fusion/data
evo_traj kitti fused.txt gt.txt measured.txt -p
```

> 需要安装evo，可以参考我的博客中的介绍：https://blog.csdn.net/u011341856/article/details/104594392?spm=1001.2014.3001.5501
