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

void loadMap(std::string vertexFilePath, std::string edgeFilePath)
// TopoMap loadMap(std::string vertexFilePath, std::string edgeFilePath)
{
    TopoMap m;
    // #pragma omp parallel sections
    {
        // #pragma omp section
        {
            std::cout << "-------------Read Vertex Data-------------" << std::endl;
            YAML::Node vertex_YN = YAML::LoadFile(vertexFilePath);
            std::cout << "Read Vertex Data from " << vertexFilePath << std::endl;
        }
        // #pragma omp section
        {
            std::cout << "-------------Read Edge Data-------------" << std::endl;
            YAML::Node edge_YN = YAML::LoadFile(edgeFilePath);
            std::cout << "Read Edge Data from " << edgeFilePath << std::endl;
        }
    }

    // return m;
}