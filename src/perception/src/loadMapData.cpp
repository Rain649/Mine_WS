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

// topoMap Classes
#include "topoMap.cpp"

// now the extraction operators for these types
void operator>>(const YAML::Node &node, Lane &l)
{
    l.id = node["lane_id"].as<int>();
    l.offset = node["offset"].as<double>();
    l.width = node["width"].as<double>();
    l.length = node["length"].as<double>();
}

void operator>>(const YAML::Node &node, Edge &e)
{
    e.id = node["id"].as<int>();
    e.entranceVertex_id = node["entranceVertex_id"].as<int>();
    e.exitVertex_id = node["exitVertex_id"].as<int>();
    const YAML::Node &lanes = node["lanes"];
    for (unsigned i = 0; i < lanes.size(); ++i)
    {
        Lane lane;
        lanes[i] >> lane;
        e.lanes_Umap.insert(std::pair<int, Lane>(lane.id, lane));
    }
}
void operator>>(const YAML::Node &node, position &p)
{
    p.x = node[0].as<double>();
    p.y = node[1].as<double>();
}

// end

void loadMap(std::string vertexFilePath, std::string edgeFilePath)
// TopoMap loadMap(std::string vertexFilePath, std::string edgeFilePath)
{
    TopoMap m;
    std::cout << "Read Vertex Data from " << vertexFilePath << std::endl;
    std::cout << "Read Edge Data from " << edgeFilePath << std::endl;
#pragma omp parallel sections
    {
#pragma omp section
        {
            YAML::Node vertex_YN = YAML::LoadFile(vertexFilePath);
            // for (unsigned i = 0; i < vertex_YN.size(); i++)
            // {
            //     Monster monster;
            //     doc[i] >> monster;
            //     std::cout << monster.name << "\n";
            // }
        }
#pragma omp section
        {
            YAML::Node edge_YN = YAML::LoadFile(edgeFilePath);
            for (unsigned i = 0; i < edge_YN.size(); ++i)
            {
                Edge edge;
                edge_YN[i] >> edge;
                std::cout << edge.id << std::endl;
            }
        }
    }

    std::cout << "-------------Read Complete-------------" << std::endl;
    // return m;
}