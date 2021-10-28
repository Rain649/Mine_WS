# 矿下感知程序使用手册

启动程序： roslaunch perception mineSimulation.launch



## 一、交叉路口检测



#### 1.源码文件：

​	intersection.cpp

#### 2.发送topic

| topic                                | type             | description            |
| ------------------------------------ | ---------------- | ---------------------- |
| "/intersection/intersectionVerified" | <std_msgs::Bool> | 判断是否位于交叉路口处 |



## 二、墙壁距离检测

#### 1.源码文件：

​	laneDetection.cpp

#### 2.发送topic

| topic                             | type                          | description                                          |
| --------------------------------- | ----------------------------- | ---------------------------------------------------- |
| "/laneDetection/leftCoefficient"  | <std_msgs::Float32MultiArray> | 左侧墙壁3阶曲线拟合系数                              |
| "/laneDetection/rightCoefficient" | <std_msgs::Float32MultiArray> | 右侧墙壁3阶曲线拟合系数                              |
| "/laneDetection/leftRange"        | <std_msgs::Float32MultiArray> | 左侧曲线拟合x轴范围（两个值：0是轴正向、1是x轴负向） |
| "/laneDetection/rightRange"       | <std_msgs::Float32MultiArray> | 右侧曲线拟合x轴范围（两个值：0是轴正向、1是x轴负向） |
| "/laneDetection/Distance"         | <std_msgs::Float32MultiArray> | 左、右侧墙壁距离（两个值：0左侧、1右侧）             |

### 三、节点定位

#### 1.源码文件：

​	navigation.cpp

​	intersectionLocation.cpp

​	topoMap.cpp

​	intersectionLocation.h 

​	topoMap.h

#### 2.发送topic

| topic                          | type                 | description                |
| ------------------------------ | -------------------- | -------------------------- |
| "/navigation/intersectionOdom" | <nav_msgs::Odometry> | 交叉路口处的定位           |
| "/navigation/intersection_id"  | <std_msgs::Int32>    | 车辆下一个经过的交叉路口ID |



### 四拓扑地图（函数）

#### 1.源码文件：

​	topoMap.h

​	topoMap.cpp

#### 2.函数

​	TopoMap loadMap(const std::string &vertexFilePath, const std::string &edgeFilePath, const std::string &pcdFilePath);

​	1.vertexFilePath为Edge.yaml路径

​	2.edgeFilePath为Vertex.yaml路径

​	3.pcdFilePath为节点点云pcd文件夹路径



