/*此程序由北京理工大学*刘仕杰*编写*/
#include <iostream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>

#define SIZE 10

using namespace std;

/********图********/   
    typedef pair<int, int> edge_Index; 

    struct Edge
    {
        int length;
        int width;
    };

    struct Vertex
    {
        Vertex() {}
        int name = INT_MAX;
        set<int> link;
    };

    class GraphLink
    {
    public:
        GraphLink()
        {
            MaxVertex = SIZE;
            NumVertex = 0;
            NumEdge = 0;
            VertexTable = new Vertex[MaxVertex];
        }

        int getVertexI(const int v)
        {
            int left = 0, right = static_cast<int> (NumVertex - 1) , middle;
            while (left <= right)
            {
                middle = (left + right) / 2;
                if (v == VertexTable[middle].name) return middle;
                else if (v < VertexTable[middle].name) right = middle - 1;
                else left = middle + 1;
            }
            return -1;
        }

        set<int> linked_Vertex(int v)
        {
            set<int> emp;
            int p = getVertexI(v);
            if (p == -1) return emp;
            return VertexTable[p].link;
        }

        void insert_Vertex(const int v)
        {
            if (NumVertex >= MaxVertex || getVertexI(v) != -1) return;
            VertexTable[NumVertex++].name = v;
        }

        void insert_Edge(const int v1, const int v2)
        {
            int p1 = getVertexI(v1);
            int p2 = getVertexI(v2);
            if (p1 == -1 || p2 == -1) return;

            VertexTable[p1].link.insert(v2);
            VertexTable[p2].link.insert(v1);

            int width(2);
            int length(5);
            Edge e;
            e.width = width;
            e.length = length;
            Edge_Map.insert(make_pair(edge_Index(min(v1, v2), max(v1, v2)), e));
        }

        void delete_Edge(const int& v1, const int& v2)
        {
            int p1 = getVertexI(v1);
            int p2 = getVertexI(v2);
            if (p1 == -1 || p2 == -1) return;

            Edge_Map.erase(edge_Index(min(v1, v2), max(v1, v2)));

            VertexTable[p1].link.erase(v2);
        }

        void delete_Vertex(const int v)
        {
            int p = getVertexI(v);
            if (p == -1) return;
            //ɾ������
            set<int> dlink = VertexTable[p].link;
            for (const int i : dlink)
            {
                delete_Edge(v, i);
                VertexTable[getVertexI(i)].link.erase(v);
            }
            //ɾ���ڵ�
            NumVertex--;
            for (size_t i = p; i < NumVertex; ++i)
                VertexTable[i] = VertexTable[i + 1];
        }

        static bool cmp(const Vertex& v1, const Vertex& v2)
        {
            return v1.name < v2.name;
        }

        void show_Edge()
        {
            cout << "------Show Edge------" << endl;
            for (const auto edge : Edge_Map)
            {
                cout << "(" << edge.first.first << ", " << edge.first.second << ") : " << 
                "length = " << edge.second.length << ", width = " << edge.second.width << endl;
            }
        }

        void show_Vertex()
        {
            cout << "------Show Vertex------" << endl;
            for (size_t i = 0; i < NumVertex; ++i)
            {
                cout << VertexTable[i].name << ", ";
            }
            cout << "\b\b." << endl << endl;

            for (size_t i = 0; i < NumVertex; ++i)
            {
                cout << VertexTable[i].name << "->(";
                for (const int j : VertexTable[i].link)
                {
                    cout << j << ", ";
                }
                if (VertexTable[i].link.empty())
                    cout << ")" << endl;
                else cout << "\b\b)" << endl;
            }
        }

        void sort()
        {
            std::sort(VertexTable, VertexTable + SIZE, cmp);
        }

        size_t size_Vertex()
        {
            return NumVertex;
        }

        size_t size_Edge()
        {
            return Edge_Map.size();
        }

        ~GraphLink()
        {
            delete[]VertexTable;
            VertexTable = NULL;
        }
    private:
        size_t MaxVertex;
        size_t NumVertex;
        size_t NumEdge;
        Vertex* VertexTable;
        map<edge_Index, Edge> Edge_Map;
    };
/********图********/

/********yaml数据********/
    struct Position
    {
        float x, y, z;
    };

    struct Intersection
    {
        int index;
        Position position;
        vector<int> adjacent_index;
        int cluster_number;
        int peak_number;
        int search_radius;
    };

    void operator >> (const YAML::Node& node, Position& position)
    {
        position.x = node[0].as<float>();
        position.y = node[1].as<float>();
        position.z = node[2].as<float>();
    }

    void operator >> (const YAML::Node& node, Intersection& intersection)
    {
        intersection.index = node["index"].as<int>();
        node["position"] >> intersection.position;
        intersection.adjacent_index = node["adjacent_index"].as<vector<int>>();
        intersection.cluster_number = node["cluster_number"].as<int>();
        intersection.peak_number = node["peak_number"].as<int>();
        intersection.search_radius = node["search_radius"].as<int>();
    }

    void print(Intersection& intersection)
    {
        cout << "Intersection Index: " << intersection.index << endl;
        cout << "Intersection position: (" << intersection.position.x << ", " <<
            intersection.position.y << ", " << intersection.position.z << ")" << endl;

        cout << "Intersection adjacent_index: (";
        for_each(intersection.adjacent_index.begin(),intersection.adjacent_index.end()-1,
            [](const auto &i){cout << i << ", ";});
        cout << intersection.adjacent_index.back() << ")\n";
        cout << "Intersection cluster_number: " << intersection.cluster_number << endl;
        cout << "Intersection peak_number: " << intersection.peak_number << endl;
        cout << "Intersection search_radius: " << intersection.search_radius << endl;
        cout << "------------------" << endl;
    }
/********yaml数据********/

int main(int argc, char **argv)
{  
    /********读取拓扑地图信息，构建拓扑地图********/
    std::string fin = "/home/lsj/dev/Mine_WS/src/preception/include/mapData.yaml";
    YAML::Node map = YAML::LoadFile(fin);
	GraphLink map_Graph;
    for(YAML::const_iterator it=map.begin();it!=map.end();++it)
    {
        Intersection j;
        *it >> j;
        print(j);
        map_Graph.insert_Vertex(j.index);
    }
    for(YAML::const_iterator it=map.begin();it!=map.end();++it)
    {
        Intersection j;
        *it >> j;
        // print(j);
        for(const auto k : j.adjacent_index)
	        map_Graph.insert_Edge(j.index, k);
    }
    map_Graph.show_Vertex();
    map_Graph.show_Edge();
    /********读取拓扑地图信息，构建拓扑地图********/
    

    return 0;
}