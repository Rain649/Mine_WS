#pragma once

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
}

void operator>>(const YAML::Node &node, perception::Vertex &v)
{
}

void operator>>(const YAML::Node &node, perception::Edge &e)
{
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