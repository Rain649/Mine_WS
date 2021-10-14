#include <iostream>
#include <string>

// own
#include "topoMap.h"

int main()
{
    std::string vertexFilePath, edgeFilePath, pcdFilePath;
    vertexFilePath = "/home/lsj/dev/Mine_WS/simu_data/Vertex.yaml";
    edgeFilePath = "/home/lsj/dev/Mine_WS/simu_data/Edge.yaml";
    pcdFilePath = "/home/lsj/dev/Mine_WS/simu_data/";

    TopoMap topoMap = loadMap(vertexFilePath, edgeFilePath, pcdFilePath);

    std::cout << "1. " << topoMap.get_cost(3) << std::endl;
    for (auto i : topoMap.get_linkedVertex_Vec(2))
    {
        std::cout << "2. " << i << std::endl;
    }
    for (auto i : topoMap.get_targetPoint(2, 3))
    {
        std::cout << "3. " << i << std::endl;
    }

    return 0;
}