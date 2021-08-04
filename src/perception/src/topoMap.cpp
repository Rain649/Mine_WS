#pragma once

// std
// #include <iostream>

// STL
#include <unordered_map>
#include <vector>

//construct topoMap
struct position
{
    float x, y;
};

struct Lane
{

    int id;
    double offset;
    double width;
    double length;

    Lane() {}
    Lane(int lane_id) : id(lane_id) {}
    Lane(int lane_id, double offset, double width, double length) : id(lane_id), offset(offset), width(width), length(length) {}
};

struct Edge
{

    int id;
    int entranceVertex_id;
    int exitVertex_id;
    std::unordered_map<int, Lane> lanes_Umap;

    Edge() {}
    Edge(int edge_id) : id(edge_id) {}
    Edge(int edge_id, int entranceVertex_id, int exitVertex_id, std::unordered_map<int, Lane> lanes_Umap) {}
};

struct FromLane
{
    int id;
    position inPoint;
    position outPoint;
    std::vector<position> path;

    FromLane() {}
    FromLane(int fromLane_id) : id(fromLane_id) {}
    FromLane(int fromLane_id, position inPoint, position outPoint, std::vector<position> path) : id(fromLane_id), inPoint(inPoint), outPoint(outPoint), path(path) {}
};

struct AccessLane
{
    int id;
    std::unordered_map<int, FromLane> fromLanes_Umap;

    AccessLane() {}
    AccessLane(int accessLane_id) : id(accessLane_id) {}
    AccessLane(int accessLane_id, std::unordered_map<int, FromLane> fromLanes_Umap) : id(accessLane_id), fromLanes_Umap(fromLanes_Umap) {}
};

struct AccessEdge
{
    int id;
    std::unordered_map<int, AccessLane> accessLanes_Umap;

    AccessEdge() {}
    AccessEdge(int accessEdge_id) : id(accessEdge_id) {}
    AccessEdge(int accessEdge_id, std::unordered_map<int, AccessLane> accessLanes_Umap) : id(accessEdge_id), accessLanes_Umap(accessLanes_Umap) {}
};

struct EntranceEdge
{
    int id;
    double angle;
    std::unordered_map<int, AccessEdge> accessEdges_Umap;

    EntranceEdge() {}
    EntranceEdge(int entranceEdge_id) : id(entranceEdge_id) {}
    EntranceEdge(int entranceEdge_id, double angle, std::unordered_map<int, AccessEdge> accessEdges_Umap) : id(entranceEdge_id), angle(angle), accessEdges_Umap(accessEdges_Umap) {}
};

struct Vertex
{

    int id;
    int branch_num;
    std::unordered_map<int, EntranceEdge> entranceEdge_Umap;

    Vertex() {}
    Vertex(int vetex_id) : id(vetex_id) {}
    Vertex(int vetex_id, int branch_num, std::unordered_map<int, EntranceEdge> entranceEdge_Umap) : id(vetex_id), branch_num(branch_num), entranceEdge_Umap(entranceEdge_Umap) {}
};

struct TopoMap
{
    std::unordered_map<int, Vertex> vertex_Umap;
    std::unordered_map<int, Edge> edge_Umap;

    TopoMap() {}
    TopoMap(std::unordered_map<int, Vertex> vertex_Umap,
            std::unordered_map<int, Edge> edge_Umap) : vertex_Umap(vertex_Umap), edge_Umap(edge_Umap) {}
};
//construct topoMap end

//search the path
void searchPath(const int &beginningVertex_id, const int &entranceEdge_id, const int &end_id, const TopoMap &topoMap)
{
    Vertex currentVertex;
    EntranceEdge currentEntranceEdge;
    double currentEntranceAngle;

    currentVertex = topoMap.vertex_Umap.at(beginningVertex_id);
    currentEntranceEdge = currentVertex.entranceEdge_Umap.at(entranceEdge_id);
    currentEntranceAngle = currentEntranceEdge.angle;
}
//search the path end
