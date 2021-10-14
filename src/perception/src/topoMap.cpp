#include "topoMap.h"

using namespace perception;

int TopoMap::get_entranceVertex_id(const int &edge_id)
{
    if (edge_Umap.count(edge_id) == 0)
        return -1;
    int entranceVertex_id;
    entranceVertex_id = edge_Umap[edge_id].entranceVertex_id;
    return entranceVertex_id;
}
int TopoMap::get_exitVertex_id(const int &edge_id)
{
    if (edge_Umap.count(edge_id) == 0)
        return -1;
    int exitVertex_id;
    exitVertex_id = edge_Umap[edge_id].exitVertex_id;
    return exitVertex_id;
}
double TopoMap::get_cost(const int &edge_id)
{
    if (edge_Umap.count(edge_id) == 0)
        return -1;
    double cost;
    cost = edge_Umap[edge_id].length;
    return cost;
}
// Vertex
LinkedInfo TopoMap::get_linkedInfo(const int &vertex1_id, const int &vertex2_id)
{
    if (vertex_Umap.count(vertex1_id) == 0 || vertex_Umap.count(vertex2_id) == 0)
        return {};
    Vertex v = vertex_Umap[vertex1_id];
    LinkedInfo lI = v.linkedInfo_Umap[vertex2_id];
    return lI;
}
double TopoMap::get_angleDiff(const int &vertex1_id, const int &vertex2_id)
{
    if (vertex_Umap.count(vertex1_id) == 0 || vertex_Umap.count(vertex2_id) == 0)
        return {};
    Vertex v = vertex_Umap[vertex1_id];
    LinkedInfo lI = v.linkedInfo_Umap[vertex2_id];
    return lI.angleDiff;
}
std::vector<int> TopoMap::get_linkedVertex_Vec(const int &vertex_id)
{
    if (vertex_Umap.count(vertex_id) == 0)
        return {};
    std::vector<int> linkedVertex_Vec;
    Vertex v = vertex_Umap[vertex_id];
    for (auto i : v.linkedInfo_Umap)
    {
        linkedVertex_Vec.push_back(i.first);
    }
    return linkedVertex_Vec;
}
std::vector<double> TopoMap::get_targetPoint(const int &vertex1_id, const int &vertex2_id)
{
    if (vertex_Umap.count(vertex1_id) == 0 || vertex_Umap.count(vertex2_id) == 0)
        return {};
    LinkedInfo lI = get_linkedInfo(vertex1_id, vertex2_id);
    return lI.targetPoint;
}
double TopoMap::get_cost(const int &vertex1_id, const int &vertex2_id)
{
    if (vertex_Umap.count(vertex1_id) == 0 || vertex_Umap.count(vertex2_id) == 0)
        return __DBL_MAX__ / 10;
    LinkedInfo lI = get_linkedInfo(vertex1_id, vertex2_id);
    double cost = edge_Umap[lI.edge_id].length;
    return cost;
}

/************Load Map************/
TopoMap loadMap(const std::string &vertexFilePath, const std::string &edgeFilePath, const std::string &pcdFilePath)
{
    std::unordered_map<int, Vertex> vertex_Umap;
    std::unordered_map<int, Edge> edge_Umap;
    std::vector<int> notExistIndex;
    std::cout
        << "Read Vertex Data from " << vertexFilePath << std::endl;
    std::cout << "Read Edge Data from " << edgeFilePath << std::endl;
#pragma omp parallel sections
    {
#pragma omp section
        {
            YAML::Node vertex_YN = YAML::LoadFile(vertexFilePath);
            for (unsigned i = 0; i < vertex_YN.size(); ++i)
            {
                Vertex vertex;
                vertex_YN[i] >> vertex;
                vertex_Umap.insert(std::pair<int, Vertex>(vertex.id, vertex));

                std::string fileName = pcdFilePath + std::to_string(vertex.id) + "_node.pcd";
                if (access(fileName.c_str(), 0))
                    notExistIndex.push_back(vertex.id);
            }
            if (!notExistIndex.empty())
            {
                std::cerr << "#########" << std::endl;
                std::cerr << "Cannot Find the Following Vertex Pcd !!!" << std::endl;
                for (int i = 0; i < notExistIndex.size(); ++i)
                    std::cout << "  " << i + 1 << ".  " << notExistIndex[i] << std::endl;
                std::cerr << "#########" << std::endl;
                exit(EXIT_FAILURE);
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
    return m;
}

void operator>>(const YAML::Node &node, LinkedInfo &lI)
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

void operator>>(const YAML::Node &node, Vertex &v)
{
    v.id = node["vertex_id"].as<int>();
    v.branch_num = node["branch_numbers"].as<int>();
    const YAML::Node &linkedInfo_Node = node["linkedInfo"];
    for (unsigned i = 0; i < linkedInfo_Node.size(); ++i)
    {
        LinkedInfo lI;
        linkedInfo_Node[i] >> lI;
        v.linkedInfo_Umap.insert(std::pair<int, LinkedInfo>(lI.linkedVertex_id, lI));
    }
}

void operator>>(const YAML::Node &node, Edge &e)
{
    e.id = node["edge_id"].as<int>();
    e.entranceVertex_id = node["entranceVertex_id"].as<int>();
    e.exitVertex_id = node["exitVertex_id"].as<int>();
    e.width = node["width"].as<double>();
    e.length = node["length"].as<double>();
}
// end