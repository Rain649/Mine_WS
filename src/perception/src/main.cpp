#include <iostream>
#include <string>

// own
#include "topoMap.cpp"
#include "loadMapData.cpp"

int main()
{
    std::string vertexFilePath, edgeFilePath;
    vertexFilePath = "/home/lsj/dev/Mine_WS/src/perception/include/Vertex.yaml";
    edgeFilePath = "/home/lsj/dev/Mine_WS/src/perception/include/Edge.yaml";

    // TopoMap topoMap = loadMap(vertexFilePath, edgeFilePath);
    loadMap(vertexFilePath, edgeFilePath);

    return 0;
}