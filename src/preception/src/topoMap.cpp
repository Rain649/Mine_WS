/*此程序由北京理工大学*刘仕杰*编写*/
#include <iostream>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <preception/topoMap_Config.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <stack>
#include <algorithm>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define SIZE 10

using namespace std;

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
        Position position;
        int cluster_Number;
        int peak_Number;
        int search_Radius;
    };

    class GraphLink
    {    
    private:
        size_t MaxVertex;
        size_t NumVertex;
        size_t NumEdge;
        Vertex* VertexTable;
        map<edge_Index, Edge> Edge_Map;
    public:
        GraphLink()
        {
            MaxVertex = SIZE;
            NumVertex = 0;
            NumEdge = 0;
            VertexTable = new Vertex[MaxVertex];
        }

        int getVertexIndex(const int v) const
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

        set<int> get_linkedVertex(const int v) const
        {
            set<int> emp;
            int p = getVertexIndex(v);
            if (p == -1) return emp;
            return VertexTable[p].link;
        }

        Vertex get_Vertex(const int v) const
        {
            int p = getVertexIndex(v);
            return VertexTable[p];
        }

        void insert_Vertex(const int v, const Position position,
         const int cluster_num, const int peak_num, const int search_radius)
        {
            if (NumVertex >= MaxVertex || getVertexIndex(v) != -1) return;
            VertexTable[NumVertex].name = v;
            VertexTable[NumVertex].position = position;
            VertexTable[NumVertex].cluster_Number = cluster_num;
            VertexTable[NumVertex].peak_Number = peak_num;
            VertexTable[NumVertex++].search_Radius = search_radius;
        }

        void insert_Edge(const int v1, const int v2)
        {
            int p1 = getVertexIndex(v1);
            int p2 = getVertexIndex(v2);
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
            int p1 = getVertexIndex(v1);
            int p2 = getVertexIndex(v2);
            if (p1 == -1 || p2 == -1) return;

            Edge_Map.erase(edge_Index(min(v1, v2), max(v1, v2)));

            VertexTable[p1].link.erase(v2);
        }

        void delete_Vertex(const int v)
        {
            int p = getVertexIndex(v);
            if (p == -1) return;
            set<int> dlink = VertexTable[p].link;
            for (const int i : dlink)
            {
                delete_Edge(v, i);
                VertexTable[getVertexIndex(i)].link.erase(v);
            }
            NumVertex--;
            for (size_t i = p; i < NumVertex; ++i)
                VertexTable[i] = VertexTable[i + 1];
        }

        static bool cmp(const Vertex& v1, const Vertex& v2)
        {
            return v1.name < v2.name;
        }

        void show_Edge() const
        {
            cout << "------Show Edge------" << endl;
            for (const auto edge : Edge_Map)
            {
                cout << "(" << edge.first.first << ", " << edge.first.second << ") : " << 
                "length = " << edge.second.length << ", width = " << edge.second.width << endl;
            }
        }

        void show_Vertex() const
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

        void show_VertexInfo(const int v) const
        {
            cout << "------Show VertexInfo------" << endl;
            int p = getVertexIndex(v);
            cout << "Vertex Name: "<< VertexTable[p].name<< endl;
            cout << "Position: (" << VertexTable[p].position.x << ", " <<
             VertexTable[p].position.y << ", " << VertexTable[p].position.z
             << ")" << endl;
            cout << "Cluster Number: "<< VertexTable[p].cluster_Number<< endl;
            cout << "Peak_Number: "<< VertexTable[p].peak_Number<< endl;
            cout << "Search Radius: "<< VertexTable[p].search_Radius<< endl;
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
    };
/********图********/

class topoMap
{
private:
    int current_Vertex{1};
    int next_Vertex{0};
    edge_Index current_Edge{0,0};

    int clusterNum{0};
    int peakNum{0};
    int segmentationRadius{0};
    
    float coeff[3]{0.5, 0.3, 0.2};

    bool intersectionVerified = false;

    stack<int> trajectory;

    pcl::PointXYZ currentPosition;

    ros::NodeHandle nh;

    ros::Subscriber subPeakNum;
    ros::Subscriber subClusterNum;
    ros::Subscriber subIntersectionVerified;
    ros::Subscriber subOdomAftMapped;
    ros::Subscriber subSegmentationRadius;

    ros::Publisher pubVertex;
public:
    topoMap() : nh("~")
    {
        subPeakNum = nh.subscribe<std_msgs::UInt8>("/intersection/peakNum", 1, &topoMap::peakNumHandler, this);
        subClusterNum = nh.subscribe<std_msgs::UInt8>("/intersection/clusterNum", 1, &topoMap::clusterNumHandler, this);
        subSegmentationRadius = nh.subscribe<std_msgs::UInt8>("/intersection/segmentationRadius", 1, &topoMap::segmentationRadiusHandler, this);
        subOdomAftMapped = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 1, &topoMap::odomAftMapped, this);
        subIntersectionVerified = nh.subscribe<std_msgs::Bool>("/intersection/intersectionVerified", 1, &topoMap::intersectionVerifiedHandler, this);
        
        pubVertex = nh.advertise<std_msgs::Int32>("Vertex", 1);
    }
    
    /********接收topics********/
        void intersectionVerifiedHandler(std_msgs::Bool msg)
        {
            intersectionVerified = msg.data;
        }

        void clusterNumHandler(std_msgs::UInt8 msg)
        {
            clusterNum = static_cast<int>(msg.data);
            ROS_DEBUG_STREAM("Cluster Number  =    " << clusterNum);
        }

        void peakNumHandler(std_msgs::UInt8 msg)
        {
            peakNum = static_cast<int>(msg.data);
            ROS_DEBUG_STREAM("Peak Number  =    " << peakNum);
        }

        void segmentationRadiusHandler(std_msgs::UInt8 msg)
        {
            segmentationRadius = static_cast<int>(msg.data);
            ROS_DEBUG_STREAM("Cluster Number  =    " << clusterNum);
        }
        
        void odomAftMapped(nav_msgs::Odometry msg)
        {
            currentPosition.x = msg.pose.pose.position.x;
            currentPosition.y = msg.pose.pose.position.y;
            currentPosition.z = msg.pose.pose.position.z;
            ROS_DEBUG_STREAM("This Key Point: x = " << currentPosition.x << " y = " << currentPosition.y << " z = " << currentPosition.z);
        }
    /********接收topics********/

    /********Vertex匹配********/
    void vertex_Matching(const GraphLink& GL)
    {
        if(intersectionVerified)
        {
            set<int> linked_Vertexes = GL.get_linkedVertex(current_Vertex);
            if(linked_Vertexes.empty()) return;
            vector<float> compatibility_Vec;
            map<float,int,greater_equal<float>> compatibility_Map;
            for(const int i:linked_Vertexes)
            {
                GL.show_VertexInfo(i);
                float compatibility = info_Matching(GL.get_Vertex(i));
                ROS_INFO("Compatibility : Name %d Value %f",i,compatibility);
                // compatibility_Vec.push_back(compatibility);
                compatibility_Map[compatibility] = i;
            }
            if(compatibility_Map.begin()->first>=0.8)
            {
                current_Vertex = compatibility_Map.begin()->second;
                ROS_ERROR("Transfer to Vertex %d",current_Vertex);
                trajectory.push(current_Vertex);
            }
        }
    }

    float info_Matching(const Vertex v)
    {
        // Position position;
        if(!clusterNum||!peakNum||!segmentationRadius) return 0;
        float res{0};
        switch(abs(clusterNum-v.cluster_Number))
        {
            case 0:
                res += coeff[0];
                break;

            case 1:
                res += 0.8*coeff[0];
                break;

            case 2:
                res += 0.5*coeff[0];
                break;

            default:
                break;
        }
        switch(abs(peakNum-v.peak_Number))
        {
            case 0:
                res += coeff[1];
                break;

            case 1:
                res += 0.8*coeff[1];
                break;

            case 2:
                res += 0.5*coeff[1];
                break;

            default:
                break;
        }
        switch(abs(segmentationRadius-v.search_Radius))
        {
            case 0:
                res += coeff[2];
                break;

            case 1:
                res += 0.8*coeff[2];
                break;

            case 2:
                res += 0.5*coeff[2];
                break;

            default:
                break;
        }
        // float res = coeff[0]*(abs(clusterNum-v.cluster_Number)/clusterNum)
        //     + coeff[1]*(abs(peakNum-v.peak_Number)/peakNum)
        //     + coeff[2]*(abs(segmentationRadius-v.search_Radius)/segmentationRadius);

        return res;
    }
    /********Vertex匹配********/

    /********Edge匹配********/
    void edge_Matching()
    {
        
        std_msgs::Int32 temp;
        temp.data = current_Vertex;
        pubVertex.publish(temp);
    }
    /********Edge匹配********/

    /********发布当前Vertex相link的Edge信息********/
    /********发布当前Vertex相link的Edge信息********/

    void getDynamicParameter()
    {
        // ros::param::get("/intersection/save_name", save_Name);
        // ros::param::get("/intersection/bool_save", save_Bool);

        ROS_DEBUG("getDynamicParameter");
    }

};


/*动态参数调节*/
    void callback(preception::topoMap_Config &config, uint32_t level)
    {
        // ROS_INFO("Reconfigure Request: %s %s",
        //          config.save_name.c_str(),
        //          config.bool_save ? "True" : "False");
    }
/*动态参数调节*/

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "topoMap");

    /*动态参数调节*/
        dynamic_reconfigure::Server<preception::topoMap_Config> server;
        dynamic_reconfigure::Server<preception::topoMap_Config>::CallbackType f;
        f = boost::bind(&callback, _1, _2);
        server.setCallback(f);
    /*动态参数调节*/

    /********读取拓扑地图信息，构建拓扑地图********/
        std::string fin = "/home/lsj/dev/Mine_WS/src/preception/include/mapData.yaml";
        YAML::Node map = YAML::LoadFile(fin);
        GraphLink map_Graph;
        for(YAML::const_iterator it=map.begin();it!=map.end();++it)
        {
            Intersection j;
            *it >> j;
            print(j);
            map_Graph.insert_Vertex(j.index, j.position,
             j.cluster_number, j.peak_number, j.search_radius);
        }
        for(YAML::const_iterator it=map.begin();it!=map.end();++it)
        {
            Intersection j;
            *it >> j;
            for(const auto k : j.adjacent_index)
                map_Graph.insert_Edge(j.index, k);
        }
        map_Graph.show_Vertex();
        map_Graph.show_Edge();
    /********读取拓扑地图信息，构建拓扑地图********/
    
    topoMap tpM;
    
    ros::Rate rate(1);
    while(ros::ok())
    {
        ros::spinOnce();

        tpM.vertex_Matching(map_Graph);
        
        rate.sleep();
    }

    return 0;
}