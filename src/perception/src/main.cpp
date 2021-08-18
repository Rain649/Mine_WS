#include <iostream>
#include <string>

// own
#include "topoMap.h"

int main()
{
    std::string vertexFilePath, edgeFilePath;
    vertexFilePath = "/home/lsj/dev/Mine_WS/src/perception/include/Vertex.yaml";
    edgeFilePath = "/home/lsj/dev/Mine_WS/src/perception/include/Edge.yaml";

    TopoMap topoMap = loadMap(vertexFilePath, edgeFilePath);

    std::cout << "1. " << topoMap.get_cost(3) << std::endl;
    for (auto i : topoMap.get_linkedVertex_Vec(2))
        std::cout << "2. " << i << std::endl;
    for (auto i : topoMap.get_targetPoint(2, 3))
        std::cout << "3. " << i << std::endl;

    return 0;
}