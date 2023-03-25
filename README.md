# find_welding_workspace
- 更新了中文说明


- TODO: 代码优化：去除无用 & 测试代码

## 仓库说明
该仓库代码用于工业箱梁生产焊接工作。本算法通过一个改良的激光-惯性里程计和点云特征提取算法来提取隔板和筋板特征，获得它们的三维坐标；然后，通过隔板和部件连接处（待焊接的地方，由设计图纸获得）的相对位置关系来计算焊接作业空间所在位置。
[pic xiangliang_real]

## 环境配置

### 仿真环境【非必须】
仿真箱梁环境来自于三维模型转化而来的STL文件。

仿真机器人使用开源项目[ROS-Academy-for-Beginners git]，上面搭载Velodyne-16线激光

但仿真环境的搭建不是必须的，我们为使用者提供了数据集。若您不想搭建仿真环境，无需编译robot_sim_demo, xiangliang, yocs_cmd_vel这三个包。
### 里程计
本算法的改良里程计基于FAST-LIO2，因此需要使用者配置AST-LIO2的环境
[fast-lio2 git]

### 特征提取
直接在工作目录下编译即可。

```
catkin_make
```

## 运行

### 仿真环境【非必须】
启动仿真和控制器

```
roslaunch robot_sim_demo sample_robot_spawn_xiangliang.launch 
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```

若您需要记录话题

```
rosbag record -O sim_xiangliang.bag /velodyne /imu_raw
```


### 里程计
由于激光-IMU外参不同，需要不同的里程计启动文件

Velodyne型激光
```
roslaunch fast_lio mapping_velodyne_xiangliang_sim.launch 
```

hesai PandarXT-16 线激光
```
roslaunch fast_lio mapping_hesai_bag2.launch 

```

### 特征提取
由于不同型号的激光传感器获得点云密度/分布不同，需要不同的阈值参数文件。

Velodyne型激光
```
roslaunch pcl_test cloudToWebs_velodyne.launch 
```

hesai PandarXT-16 线激光
```
roslaunch pcl_test cloudToWebs_PandarXT-16_bag2.launch
```