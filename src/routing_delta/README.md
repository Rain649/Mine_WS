# 矿下拓扑地图规划

- /include 中的 topoMap.h 和 /src 中的 topoMap.cpp 需要与其他包保持一致
- roslaunch routing_delta routing_delta.launch

## 输入

- 拓扑地图文件
- 起点和终点 id，起点为车辆后方第一个节点
- 是否位于路口的旗标，/intersectionDetection/intersectionVerified

## 输出

- 规划路径的节点序列，/pathArray
- 路口的出口坐标，/target_fork

