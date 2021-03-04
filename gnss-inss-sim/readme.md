这里只提供我所使用的[gnss-ins-sim](https://github.com/Aceinna/gnss-ins-sim)文件和运动参数文件!

> `gnss-ins-sim`是一个用于仿真imu、gps、磁力计数据的软件

## 1.准备工作

a. 如果想要运行`my_test.py`，你需要下载完整的[gnss-ins-sim](https://github.com/Aceinna/gnss-ins-sim)

```bash
git clone https://github.com/Aceinna/gnss-ins-sim.git
```

b. 下载下来之后，将`my_test.csv`拷贝到`gnss-ins-sim/demo_motion_def_files/`

```bash
cp my_test.csv gnss-ins-sim/demo_motion_def_files/
```

c. 将`my_test.py`拷贝到`gnss-ins-sim/`

```bash
cp my_test.py gnss-ins-sim
```

 ## 2.运行

```bash
cd gnss-ins-sim
python my_test.py
```

> 执行完上一步命令之后，会在`gnss-ins-sim/demo_saved_data`文件夹下生成一个以时间信息为名称的文件夹，里面的数据包含了imu、gps所需要的所有数据

## 3.注意

- [gnss-ins-sim](https://github.com/Aceinna/gnss-ins-sim)是一个用于仿真imu、gps、磁力计数据的软件，是一个非常不错的小工具，使用起来非常容易，建议你看一下它的[README介绍文件](https://github.com/Aceinna/gnss-ins-sim#gnss-ins-sim)。
- 一定要注意坐标系之间的关系，imu是body frame，gps位置是lla形式，是在NED(坐标系)，gps数据通过`GeographicLib`库转换之后对应的是ENU(东北天)坐标系，这之间的坐标关系，你一定要理清楚，不然很容易就会出错。

