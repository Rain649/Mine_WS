#pragma once

// std
// #include <iostream>

// STL
#include <unordered_map>
#include <vector>

struct position
{
    float x, y;
};

class Lane
{
private:
    int id;
    double offset;
    double width;
    double length;

public:
    Lane(int lane_id) : id(lane_id) {}
    Lane(int lane_id, double offset, double width, double length) : id(lane_id), offset(offset), width(width), length(length) {}

    ///* get Lane's offset
    double get_offset() const
    {
        return offset;
    }
    ///* get Lane's width
    double get_width() const
    {
        return width;
    }
    ///* get Lane's length
    double get_length() const
    {
        return length;
    }
};

class Edge
{
private:
    int id;
    int entranceVertex_id;
    int exitVertex_id;
    std::unordered_map<int, Lane> lanes_Umap;

public:
    Edge(int edge_id) : id(edge_id) {}
    Edge(int edge_id, int entranceVertex_id, int exitVertex_id, std::unordered_map<int, Lane> lanes_Umap) {}

    ///* get edge's entranceVertex_id
    int get_entranceVertex_id() const
    {
        return entranceVertex_id;
    }
    ///* get edge's exitVertex_id
    int get_exitVertex_id() const
    {
        return exitVertex_id;
    }
    ///* get edge's lanes_Umap
    std::unordered_map<int, Lane> get_lanes_Umap() const
    {
        return lanes_Umap;
    }
};

class FromLane
{
private:
    int id;
    position inPoint;
    position outPoint;
    std::vector<position> path;

public:
    FromLane(int fromLane_id) : id(fromLane_id) {}
    FromLane(int fromLane_id, position inPoint, position outPoint, std::vector<position> path) : id(fromLane_id), inPoint(inPoint), outPoint(outPoint), path(path) {}

    ///* get FromLane's inPoint
    position get_inPoint() const
    {
        return inPoint;
    }
    ///* get FromLane's outPoint
    position get_outPoint() const
    {
        return outPoint;
    }
    ///* get FromLane's path
    std::vector<position> get_path() const
    {
        return path;
    }
};

class AccessLane
{
private:
    int id;
    std::unordered_map<int, FromLane> fromLanes_Umap;

public:
    AccessLane(int accessLane_id) : id(accessLane_id) {}
    AccessLane(int accessLane_id, std::unordered_map<int, FromLane> fromLanes_Umap) : id(accessLane_id), fromLanes_Umap(fromLanes_Umap) {}

    ///* get AccessLane's fromLanes_Umap
    std::unordered_map<int, FromLane> get_fromLanes_Umap() const
    {
        return fromLanes_Umap;
    }
};

class AccessEdge
{
private:
    int id;
    std::unordered_map<int, AccessLane> accessLanes_Umap;

public:
    AccessEdge(int accessEdge_id) : id(accessEdge_id) {}
    AccessEdge(int accessEdge_id, std::unordered_map<int, AccessLane> accessLanes_Umap) : id(accessEdge_id), accessLanes_Umap(accessLanes_Umap) {}

    ///* get AccessEdge's accessLanes_Umap
    std::unordered_map<int, AccessLane> get_accessLanes_Umap() const
    {
        return accessLanes_Umap;
    }
};

class Vertex
{
private:
    int id;
    int branch_num;
    std::unordered_map<int, AccessEdge> accessEdge_Umap;

public:
    Vertex(int vetex_id) : id(vetex_id) {}
    Vertex(int vetex_id, int branch_num, std::unordered_map<int, AccessEdge> accessEdge_Umap) : id(vetex_id), branch_num(branch_num), accessEdge_Umap(accessEdge_Umap) {}

    ///* get vertex's branch_num
    int get_branch_num() const
    {
        return branch_num;
    }
    ///* get vertex's accessEdge_Umap
    std::unordered_map<int, AccessEdge> get_accessEdge_Umap() const
    {
        return accessEdge_Umap;
    }
};

class TopoMap
{
private:
    std::unordered_map<int, Vertex> vertex_Umap;
    std::unordered_map<int, Edge> edge_Umap;

public:
    TopoMap() {}
    TopoMap(std::unordered_map<int, Vertex> vertex_Umap,
            std::unordered_map<int, Edge> edge_Umap) : vertex_Umap(vertex_Umap), edge_Umap(edge_Umap) {}

    ///* get TopoMap's vertex_Umap
    std::unordered_map<int, Vertex> get_vertex_Umap() const
    {
        return vertex_Umap;
    }
    ///* get TopoMap's edge_Umap
    std::unordered_map<int, Edge> get_edge_Umap() const
    {
        return edge_Umap;
    }
};
