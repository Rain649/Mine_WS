// #pragma once

#ifndef TopoMap_H
#define TopoMap_H

#include <array>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string>
#include <omp.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <unistd.h>

inline void radianTransform(double &radian);
namespace perception
{

    struct pair_hash
    {
        template <class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2> &p) const
        {
            auto h1 = std::hash<T1>{}(p.first);
            auto h2 = std::hash<T2>{}(p.second);
            return h1 ^ h2;
        }
    };

    ///* Define link information
    struct LinkedInfo
    {
        int linkedVertex_id;
        int edge_id;
        double angleDiff;
        std::array<double, 3> startPoint;
        std::vector<double> targetPoint; // x, y, phi

        LinkedInfo() {}
        LinkedInfo(int _linkedVertex_id, int _edge_id, double _angleDiff, std::array<double, 3> _startPoint, std::vector<double> _targetPoint) : linkedVertex_id(_linkedVertex_id), edge_id(_edge_id), angleDiff(_angleDiff), startPoint(_startPoint), targetPoint(_targetPoint) {}
    };

    struct Vertex
    {
        int id;
        int branch_num;
        ///* 可以连接的相邻节点信息
        std::unordered_map<int, LinkedInfo> linkedInfo_Umap;
        ///* 路径Umap
        std::unordered_map<std::pair<int, int>, std::vector<std::array<double, 3>> *, pair_hash> path_Umap;

        Vertex() {}
        Vertex(int _vetex_id, int _branch_num, std::unordered_map<int, LinkedInfo> _linkedInfo_Umap, std::unordered_map<std::pair<int, int>, std::vector<std::array<double, 3>> *, pair_hash> _path_Umap) : id(_vetex_id), branch_num(_branch_num), linkedInfo_Umap(_linkedInfo_Umap), path_Umap(_path_Umap) {}
    };

    struct Edge
    {
        int id;
        int entranceVertex_id;
        int exitVertex_id;

        double length, width;
        // double cost;

        Edge() {}
        Edge(int _edge_id, int _entranceVertex_id, int _exitVertex_id, double _length, double _width) : id(_edge_id), exitVertex_id(_exitVertex_id), length(_length), width(_width) {}
    };
}

class TopoMap
{
private:
    std::unordered_map<int, perception::Vertex> vertex_Umap;
    std::unordered_map<int, perception::Edge> edge_Umap;

public:
    // Edge
    int get_entranceVertex_id(const int &edge_id);
    int get_exitVertex_id(const int &edge_id);
    double get_cost(const int &edge_id);
    std::vector<std::array<double, 3>> get_path(const int &incomingVertex_id, const int &currentVertex_id, const int &connectingVertex_id);
    // Vertex
    perception::LinkedInfo get_linkedInfo(const int &vertex1_id, const int &vertex2_id);
    //弧度
    double get_angleDiff(const int &vertex1_id, const int &vertex2_id);
    //起始点坐标
    std::array<double, 3> get_startPoint(const int &vertex1_id, const int &vertex2_id);
    std::vector<int> get_linkedVertex_Vec(const int &vertex_id);
    std::vector<double> get_targetPoint(const int &vertex1_id, const int &vertex2_id);
    double get_cost(const int &vertex1_id, const int &vertex2_id);

    TopoMap() {}
    TopoMap(std::unordered_map<int, perception::Vertex> _vertex_Umap, std::unordered_map<int, perception::Edge> _edge_Umap) : vertex_Umap(_vertex_Umap), edge_Umap(_edge_Umap) {}
};

void operator>>(const YAML::Node &node, perception::LinkedInfo &lI);
void operator>>(const YAML::Node &node, perception::Vertex &v);
void operator>>(const YAML::Node &node, perception::Edge &e);

TopoMap loadMap(const std::string &vertexFilePath, const std::string &edgeFilePath, const std::string &pcdFilePath);

#endif
