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
//
void operator>>(const YAML::Node &node, position &p)
{
    p.x = node[0].as<double>();
    p.y = node[1].as<double>();
}

void operator>>(const YAML::Node &node, FromLane &fl)
{
    fl.id = node["fromLane_id"].as<int>();
    node["inPoint"] >> fl.inPoint;
    node["outPoint"] >> fl.outPoint;
    const YAML::Node &path = node["path"];
    for (unsigned i = 0; i < path.size(); ++i)
    {
        position p;
        path[i]["point"] >> p;
        fl.path.push_back(p);
    }
}

void operator>>(const YAML::Node &node, AccessLane &al)
{
    al.id = node["accessLane_id"].as<int>();
    const YAML::Node &fromLanes = node["fromLanes"];
    for (unsigned i = 0; i < fromLanes.size(); ++i)
    {
        FromLane fl;
        fromLanes[i] >> fl;
        al.fromLanes_Umap.insert(std::pair<int, FromLane>(fl.id, fl));
    }
}

void operator>>(const YAML::Node &node, AccessEdge &ae)
{
    ae.id = node["accessEdge_id"].as<int>();
    const YAML::Node &accessLanes = node["accessLanes"];
    for (unsigned i = 0; i < accessLanes.size(); ++i)
    {
        AccessLane al;
        accessLanes[i] >> al;
        ae.accessLanes_Umap.insert(std::pair<int, AccessLane>(al.id, al));
    }
}

void operator>>(const YAML::Node &node, EntranceEdge &ee)
{
    ee.id = node["entranceEdge_id"].as<int>();
    ee.angle = node["angle"].as<double>();
    const YAML::Node &accessEdges = node["accessEdges"];
    for (unsigned i = 0; i < accessEdges.size(); ++i)
    {
        AccessEdge ae;
        accessEdges[i] >> ae;
        ee.accessEdges_Umap.insert(std::pair<int, AccessEdge>(ae.id, ae));
    }
}

void operator>>(const YAML::Node &node, Vertex &v)
{
    v.id = node["vertex_id"].as<int>();
    v.branch_num = node["branch_numbers"].as<int>();
    const YAML::Node &entranceEdges = node["entranceEdges"];
    for (unsigned i = 0; i < entranceEdges.size(); ++i)
    {
        EntranceEdge ee;
        entranceEdges[i] >> ee;
        v.entranceEdge_Umap.insert(std::pair<int, EntranceEdge>(ee.id, ee));
    }
}
// end

void loadMap(std::string vertexFilePath, std::string edgeFilePath)
// TopoMap loadMap(std::string vertexFilePath, std::string edgeFilePath)
{
    std::unordered_map<int, Vertex> vertex_Umap;
    std::unordered_map<int, Edge> edge_Umap;
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
                Vertex vertex;
                vertex_YN[i] >> vertex;
                vertex_Umap.insert(std::pair<int, Vertex>(vertex.id, vertex));
            }
        }
#pragma omp section
        {
            YAML::Node edge_YN = YAML::LoadFile(edgeFilePath);
            for (unsigned i = 0; i < edge_YN.size(); ++i)
            {
                Edge edge;
                edge_YN[i] >> edge;
                edge_Umap.insert(std::pair<int, Edge>(edge.id, edge));
                // std::cout << edge.id << std::endl;
            }
        }
    }
    TopoMap m(vertex_Umap, edge_Umap);

    std::cout << "-------------Read Complete-------------" << std::endl;
    // return m;
}