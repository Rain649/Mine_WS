# 矿下感知定位程序使用手册

## 一、程序介绍：

```
本程序为矿下感知定位程序，具备激光雷达点云融合、交叉路口检测、车道线拟合、交叉路口定位的功能。在gazebo中搭建了地图和车辆模型，搭配规划、控制算法，可不依赖于定位的基础设施完成单车智能驾驶。
```

## 1.点云处理（去除车辆、地面点）

#### 1).源码文件：

```
lidarCloudProcess.cpp
```

#### 2).发送topic

| topic                               | type                                              | description              |
| ----------------------------------- | ------------------------------------------------- | ------------------------ |
| "/lidarCloudProcess/cloud_Combined" | [sensor_msgs::PointCloud2](sensor_msgs::PointCloud2) | 去除地面点、车辆点的点云 |

## 2.交叉路口检测

#### 1).源码文件：

```
intersectionDetection.cpp
```

#### 2).发送topic

| topic                                         | type                          | description            |
| --------------------------------------------- | ----------------------------- | ---------------------- |
| "/intersectionDetection/intersectionVerified" | [std_msgs::Bool](std_msgs::Bool) | 判断是否位于交叉路口处 |

## 3.车道线拟合

#### 1).源码文件：

```
laneDetection.cpp
```

#### 2).发送topic

| topic                             | type                                                    | description                                          |
| --------------------------------- | ------------------------------------------------------- | ---------------------------------------------------- |
| "/laneDetection/leftCoefficient"  | [std_msgs::Float32MultiArray](std_msgs::Float32MultiArray) | 左侧墙壁3阶曲线拟合系数                              |
| "/laneDetection/rightCoefficient" | [std_msgs::Float32MultiArray](std_msgs::Float32MultiArray) | 右侧墙壁3阶曲线拟合系数                              |
| "/laneDetection/leftRange"        | [std_msgs::Float32MultiArray](std_msgs::Float32MultiArray) | 左侧曲线拟合x轴范围（两个值：0是轴正向、1是x轴负向） |
| "/laneDetection/rightRange"       | [std_msgs::Float32MultiArray](std_msgs::Float32MultiArray) | 右侧曲线拟合x轴范围（两个值：0是轴正向、1是x轴负向） |
| "/laneDetection/Distance"         | [std_msgs::Float32MultiArray](std_msgs::Float32MultiArray) | 左、右侧墙壁距离（两个值：0左侧、1右侧）             |

## 4.节点定位

#### 1).源码文件：

```
navigation.cpp
```

```
intersectionLocation.cpp
```

```
topoMap.cpp
```

```
intersectionLocation.h
```

```
topoMap.h
```

#### 2).发送topic

| topic                          | type                                  | description                |
| ------------------------------ | ------------------------------------- | -------------------------- |
| "/navigation/intersectionOdom" | [nav_msgs::Odometry](nav_msgs::Odometry) | 交叉路口处的定位           |
| "/navigation/intersection_id"  | [std_msgs::Int32](std_msgs::Int32)       | 车辆下一个经过的交叉路口ID |

#### 3).接收topic

| topic        | type                                                | description                                                                                                  |
| ------------ | --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------ |
| "/pathArray" | [std_msgs::Int32MultiArray](std_msgs::Int32MultiArray) | 需要获得规划的路径，路径数组第一个值需要设定为第一个经过节点前最后经过的节点，若是起始位置在1号节点，则需补0 |

## 5.拓扑地图（函数）

#### 1).源码文件：

```
topoMap.h
```

```
topoMap.cpp
```

#### 2).函数

```
TopoMap loadMap(const std::string &vertexFilePath, const std::string &edgeFilePath, const std::string &pcdFilePath);
```

```
1.vertexFilePath为Edge.yaml路径
```

```
2.edgeFilePath为Vertex.yaml路径
```

```
3.pcdFilePath为节点点云pcd文件夹路径
```

## 二、安装与卸载：

```
直接将包放在工作空间编译。
```

## 三、依赖介绍

| 依赖项   | 版本 |
| -------- | ---- |
| PCL      | 1.8  |
| EIGEN    | 3    |
| yaml-cpp | 最新 |

## 四、编译与运行

直接使用catkin_make编译

启动程序： roslaunch perception mineSimulation.launch

## 五、参数介绍

### 0.公用参数

| 名称              | 描述                                |
| ----------------- | ----------------------------------- |
| pointCloud_topic_ | 多个激光雷达融合、处理后的点云topic |
| frame_id_         | 车辆坐标系frame_id                  |

### 1.gazebo参数

| 名称         | 描述                          |
| ------------ | ----------------------------- |
| paused       | 暂停                          |
| use_sim_time | 使用仿真时间                  |
| gui          | 开启一个带有用户界面的Gazebo  |
| debug        | 开启一个debug模式下的gzserver |

### 2.车辆参数

| 名称              | 描述          |
| ----------------- | ------------- |
| model             | 车辆xacro模型 |
| robot_description | 机器人参数    |

### 3.gazebo参数

| 名称              | 描述              |
| ----------------- | ----------------- |
| lidarTopic_left_  | 左侧激光雷达topic |
| lidarTopic_right_ | 右侧激光雷达topic |
| lidarTopic_top_   | 顶部激光雷达topic |

### 4.交叉路口检测参数

| 名称 | 描述 |
| ---- | ---- |
|      |      |

### 5.车道线拟合参数

| 名称           | 描述         |
| -------------- | ------------ |
| order          | 拟合曲线阶数 |
| minClusterSize | 最小聚类数量 |
| clusterRadius  | 聚类搜索半径 |
| passX_min      | x轴最小值    |
| passZ_min      | z轴最小值    |
| passZ_max      | z轴最大值    |

### 6.定位导航参数

| 名称           | 描述                |
| -------------- | ------------------- |
| simu_data_path | 局部地图点云pcd路径 |

## 六、测试

1. 启动roscore：roscore
2. 启动仿真模型及感知定位程序： roslaunch perception mineSimulation.launch
3. （测试）单独启动定位程序： rosrun perception navigation
4. 启动局部路径规划程序：roslaunch planner_underground_delta planner_underground_delta.launch
5. 启动控制程序：roslaunch control_underground_delta control_underground_delta.launch
6. 启动全局路径规划程序：roslaunch routing_delta routing_delta.launch
7. 输入起始节点id和目标节点id
8. gazebo点击开始运行按钮
9. (可选)启动车辆控制程序：rosrun teleop_twist_keyboard teleop_twist_keyboard.py
