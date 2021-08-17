// std
#include <iostream>
#include <stdio.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <omp.h>

// STL
#include <unordered_map>
#include <vector>

// topoMap Class
#include "topoMap.h"

// now the extraction operators for these types
void operator>>(const YAML::Node &node, perception::LinkedInfo &lI)
{
    lI.linkedVertex_id = node["linkedVertex_id"].as<int>();
    lI.edge_id = node["edge_id"].as<int>();
    lI.angleDiff = node["angleDiff"].as<double>();
    YAML::Node targetPoint_node = node["targetPoint"];
    for (unsigned i = 0; i < targetPoint_node.size(); ++i)
    {
        lI.targetPoint.push_back(targetPoint_node[i].as<double>());
    }
}

void operator>>(const YAML::Node &node, perception::Vertex &v)
{
    v.id = node["vertex_id"].as<int>();
    v.branch_num = node["branch_numbers"].as<int>();
    const YAML::Node &linkedInfo_Node = node["linkedInfo"];
    for (unsigned i = 0; i < linkedInfo_Node.size(); ++i)
    {
        perception::LinkedInfo lI;
        linkedInfo_Node[i] >> lI;
        v.linkedInfo_Umap.insert(std::pair<int, perception::LinkedInfo>(lI.linkedVertex_id, lI));
    }
}

void operator>>(const YAML::Node &node, perception::Edge &e)
{
    e.id = node["edge_id"].as<int>();
    e.entranceVertex_id = node["entranceVertex_id"].as<int>();
    e.exitVertex_id = node["exitVertex_id"].as<int>();
    e.width = node["width"].as<double>();
    e.length = node["length"].as<double>();
}
// end

TopoMap loadMap(const std::string &vertexFilePath, const std::string &edgeFilePath)
{
    std::unordered_map<int, perception::Vertex> vertex_Umap;
    std::unordered_map<int, perception::Edge> edge_Umap;
    std::cout
        << "Read Vertex Data from " << vertexFilePath << std::endl;
    std::cout << "Read Edge Data from " << edgeFilePath << std::endl;
#pragma omp parallel sections
    {
#pragma omp section
        {
            YAML::Node vertex_YN = YAML::LoadFile(vertexFilePath);
            for (unsigned i = 0; i < vertex_YN.size(); i++)
            {
                perception::Vertex vertex;
                vertex_YN[i] >> vertex;
                vertex_Umap.insert(std::pair<int, perception::Vertex>(vertex.id, vertex));
            }
        }
#pragma omp section
        {
            YAML::Node edge_YN = YAML::LoadFile(edgeFilePath);
            for (unsigned i = 0; i < edge_YN.size(); ++i)
            {
                perception::Edge edge;
                edge_YN[i] >> edge;
                edge_Umap.insert(std::pair<int, perception::Edge>(edge.id, edge));
                // std::cout << edge.id << std::endl;
            }
        }
    }
    TopoMap m(vertex_Umap, edge_Umap);

    std::cout << "-------------Read Complete-------------" << std::endl;
    return m;
}