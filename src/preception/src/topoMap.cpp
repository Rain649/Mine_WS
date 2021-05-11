/*此程序由北京理工大学*刘仕杰*编写*/
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <thread>
#include <dynamic_reconfigure/server.h>
#include <preception/topoMap_Config.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <stack>
#include <algorithm>
#include <thread>
#include <mutex>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/kdtree/kdtree_flann.h>    //kdtree搜索

#define SIZE 10

using namespace std;

std::string filePath = "/home/lsj/dev/Mine_WS/data/";

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
        float begin_Angle;
        float end_Angle;
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

        void insert_Edge(const int v1, const int v2, const float begin_Angle = 0, const float end_Angle = 0)
        {
            auto it = Edge_Map.find(edge_Index(min(v1, v2), max(v1, v2)));
            if(it!=Edge_Map.end())
            {
                it->second.begin_Angle = begin_Angle;
                it->second.end_Angle = end_Angle;
                return;
            }

            int p1 = getVertexIndex(v1);
            int p2 = getVertexIndex(v2);
            if (p1 == -1 || p2 == -1) return;

            VertexTable[p1].link.insert(v2);
            VertexTable[p2].link.insert(v1);

            int width(2);
            int length(5);
            Edge e;
            e.width = width;
            e.begin_Angle = begin_Angle;
            e.end_Angle = end_Angle;
            Edge_Map.insert(make_pair(edge_Index(min(v1, v2), max(v1, v2)), e));
        }

        Edge get_Edge(const int v1, const int v2) const
        {
            const auto iter = Edge_Map.find(edge_Index(min(v1, v2), max(v1, v2)));
            if(iter!=Edge_Map.end()) return iter->second;
            else return {};
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
                "length = " << edge.second.length << ", width = " << edge.second.width <<
                ", begin_Angle = " << edge.second.begin_Angle << ", end_Angle = " << edge.second.end_Angle << endl;
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
    int current_Vertex{5};
    // int next_Vertex{0};
    int v1{0}, v2{1};   //设置左右窗口
    // edge_Index current_Edge{0,0};

    int clusterNum{0};
    int peakNum{0};
    int segmentationRadius{0};
    
    // float coeff[3]{0.6, 0.3, 0.1};
    float coeff[3]{1, 0, 0};

    bool intersectionVerified = false;
    bool transferComplete = false;
    bool intersectionVerifiedPre = false;

    std::string fin = "/home/lsj/dev/Mine_WS/src/preception/include/ndtData.yaml";
    
    stack<int> trajectory;

    map<int,int> currentVertex_Times;

    pcl::visualization::PCLVisualizer viewer {"ndt"};   // 初始化点云可视化界面
    
    pcl::PointXYZ currentPosition;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOrigin;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;    //初始化正态分布变换（NDT）

    ros::NodeHandle nh;

    ros::Subscriber subPeakNum;
    ros::Subscriber subClusterNum;
    ros::Subscriber subIntersectionVerified;
    ros::Subscriber subOdomAftMapped;
    ros::Subscriber subSegmentationRadius;
    ros::Subscriber subCloudOrigin;

    ros::Publisher pubVertex;

    mutex mtx;
public:
    topoMap() : nh()
    {
        subPeakNum = nh.subscribe<std_msgs::UInt8>("/peakNum", 1, &topoMap::peakNumHandler, this);
        subClusterNum = nh.subscribe<std_msgs::UInt8>("/clusterNum", 1, &topoMap::clusterNumHandler, this);
        subSegmentationRadius = nh.subscribe<std_msgs::UInt8>("/segmentationRadius", 1, &topoMap::segmentationRadiusHandler, this);
        subOdomAftMapped = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 1, &topoMap::odomAftMapped, this);
        subIntersectionVerified = nh.subscribe<std_msgs::Bool>("/intersectionVerified", 1, &topoMap::intersectionVerifiedHandler, this);
        subCloudOrigin = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_ma", 1, &topoMap::cloudOriginHandler, this);
        
        pubVertex = nh.advertise<std_msgs::Int32>("Vertex", 1);

        viewer.createViewPort(0.0, 0.0, 0.5, 1, v1); 
        viewer.setBackgroundColor(0, 0, 0, v1);
        viewer.createViewPort(0.5, 0.0, 1, 1, v2);
        viewer.setBackgroundColor(0.5, 0.5, 0.5, v2);
        viewer.addCoordinateSystem (5);
        viewer.initCameraParameters();
        viewer.setCameraPosition(30,40,50,-3,-4,-5,0);

        cloudOrigin.reset(new pcl::PointCloud<pcl::PointXYZ>);
        target_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        output_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    
    /********读取数据********/
        void configLoad()
        {
        }
    /********读取数据********/

    /********接收topics********/
        void intersectionVerifiedHandler(const std_msgs::Bool msg)
        {
            intersectionVerifiedPre = intersectionVerified;
            intersectionVerified = msg.data;
        }

        void clusterNumHandler(const std_msgs::UInt8 msg)
        {
            clusterNum = static_cast<int>(msg.data);
            ROS_DEBUG_STREAM("Cluster Number  =    " << clusterNum);
        }

        void peakNumHandler(const std_msgs::UInt8 msg)
        {
            peakNum = static_cast<int>(msg.data);
            ROS_DEBUG_STREAM("Peak Number  =    " << peakNum);
        }

        void segmentationRadiusHandler(const std_msgs::UInt8 msg)
        {
            segmentationRadius = static_cast<int>(msg.data);
            ROS_DEBUG_STREAM("Cluster Number  =    " << clusterNum);
        }
        
        void odomAftMapped(const nav_msgs::Odometry msg)
        {
            currentPosition.x = msg.pose.pose.position.x;
            currentPosition.y = msg.pose.pose.position.y;
            currentPosition.z = msg.pose.pose.position.z;
            ROS_DEBUG_STREAM("This Key Point: x = " << currentPosition.x << " y = " << currentPosition.y << " z = " << currentPosition.z);
        }

        void cloudOriginHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
        {
            pcl::fromROSMsg(*msg, *cloudOrigin);
            ROS_DEBUG_STREAM("cloudOrigin  " << cloudOrigin->size());
        }
    /********接收topics********/

    /********Vertex匹配********/
    void vertex_Matching(const GraphLink& GL)
    {
        if(intersectionVerified&&!transferComplete)
        {
            set<int> linked_Vertexes = GL.get_linkedVertex(current_Vertex);
            if(linked_Vertexes.empty()) return;
            map<float,int,greater_equal<float>> compatibility_Map;

            if(!intersectionVerifiedPre)
                for(const int i:linked_Vertexes)
                    GL.show_VertexInfo(i);
            cout << clusterNum << " " << peakNum << " " << segmentationRadius << endl;

            map<int,double> transfer_Map;
            for(const int i:linked_Vertexes)
            {
                float compatibility = info_Matching(GL.get_Vertex(i));
                ROS_INFO("Compatibility : Name %d Value %f",i,compatibility);
                compatibility_Map[compatibility] = i;
                if(compatibility>=0.8)
                {
                    ROS_ERROR("PCD MATCHING!!!!!!!");
                    double score = pcd_Matching(i, GL.get_Edge(current_Vertex, i));
                    cout << "score = " << score << endl;
                    if(score < 0.6)
                        transfer_Map[score] = i;
                }
            }
            if(!transfer_Map.empty())
            {    
                current_Vertex = transfer_Map.begin()->second;
                transferComplete = true;
                ROS_ERROR("Transfer to Vertex %d",current_Vertex);
            }
            if(trajectory.empty()||trajectory.top()!=current_Vertex)
                trajectory.push(current_Vertex);
        }
        else if(!intersectionVerified&&intersectionVerifiedPre)
        { 
            currentVertex_Times.clear();
            transferComplete = false;
        }
    }

    float info_Matching(const Vertex v)
    {
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
 
    double pcd_Matching(const int target_Index,const Edge this_Edge)
    {
        YAML::Node config = YAML::LoadFile(fin);

        float maximumIterations = config["maximumIterations"].as<float>();
        float resolution = config["resolution"].as<float>();
        float stepSize = config["stepSize"].as<float>();
        float transformationEpsilon = config["transformationEpsilon"].as<float>();
        float yaw_pre = config["yaw_pre"].as<float>()*M_PI/180;
        float x_pre = config["x_pre"].as<float>();
        float y_pre = config["y_pre"].as<float>();
        float menu_Bool = config["menu_bool"].as<bool>();

        //加载目标点云pcd
        std::string fileName = filePath + std::to_string(target_Index) + "_whole.pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *target_cloud) == -1)
        {
            PCL_ERROR ("Couldn't read file target_cloud.pcd \n");
            return (-1);
        }

        for(const auto p:*target_cloud)
        {
            pcl::PointXYZ p1;
            p1.x = p.z;
            p1.y = p.x;
            p1.z = p.y;
            filtered_cloud->push_back(p1);
        }
        filtered_cloud->swap(*target_cloud);
        filtered_cloud->clear();
        
        *filtered_cloud += *target_cloud;
        target_cloud->clear();
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*filtered_cloud, *target_cloud, indices);
        filtered_cloud->clear();

        pcl::removeNaNFromPointCloud(*cloudOrigin, *cloudOrigin, indices);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloudOrigin); //设置要搜索的点云，建立KDTree
        std::vector<int> pointIdxRadiusSearch;  //保存每个近邻点的索引
        std::vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
        if (kdtree.radiusSearch(pcl::PointXYZ(0,0,0), 25, pointIdxRadiusSearch, pointRadiusSquaredDistance)==0)
            return -1;
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            filtered_cloud->push_back(cloudOrigin->points[pointIdxRadiusSearch[i]]);

        //将输入的扫描过滤到原始尺寸的大概10%以提高匹配的速度。
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
        approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
        approximate_voxel_filter.setInputCloud (filtered_cloud);
        approximate_voxel_filter.filter (*filtered_cloud);

        /*设置依赖尺度NDT参数*/
        //为终止条件设置最小转换差异
        ndt.setTransformationEpsilon (transformationEpsilon);//定义了[x,y,z,roll,pitch,yaw]在配准中的最小递增量，一旦递增量小于此限制，配准终止
        //为More-Thuente线搜索设置最大步长，步长越大迭代越快，但也容易导致错误
        ndt.setStepSize (stepSize);
        //设置NDT网格结构的分辨率（VoxelGridCovariance）
        ndt.setResolution (resolution);//ND体素的大小，单位为m,越小越准确，但占用内存越多
        //设置匹配迭代的最大次数
        ndt.setMaximumIterations (maximumIterations);
        //设置要配准的点云
        ndt.setInputCloud (filtered_cloud);
        //设置点云配准目标
        ndt.setInputTarget (target_cloud);
        //设置使用机器人测距法得到的初始对准估计结果

        float yaw_init;
        if(!menu_Bool)
        {
            if(current_Vertex<target_Index)
                yaw_init = this_Edge.end_Angle;
            else yaw_init = this_Edge.begin_Angle;
            ROS_INFO_STREAM("end Angle = " << yaw_init);
        }
        else yaw_init = yaw_pre;
        Eigen::AngleAxisf init_rotation (yaw_init, Eigen::Vector3f::UnitZ());
        Eigen::Translation3f init_translation (x_pre, y_pre, 0);
        Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
        //计算需要的刚体变换以便将输入的点云匹配到目标点云
        ndt.align (*output_cloud, init_guess);//这一步是将降采样之后的点云经过变换后得到output_cloud
        double res = ndt.getFitnessScore ();
        if(ndt.hasConverged ())
        ROS_INFO_STREAM("Normal Distributions Transform Score = " << res);
        
        //ndt.getFinalTransformation (）即最终变换
        Eigen::Matrix4f transformation = ndt.getFinalTransformation();
        //使用创建的变换对未过滤的输入点云进行变换
        pcl::transformPointCloud (*filtered_cloud, *output_cloud, ndt.getFinalTransformation ());
        Eigen::Matrix3f transformation_3f;
        for(int i=0;i<3;++i)
            for(int j=0;j<3;++j)
                transformation_3f(i,j) = transformation(i,j);
        Eigen::Vector3f eulerAngle = transformation_3f.eulerAngles(0,1,2)*180/M_PI;
        cout << "roll, pitch, yaw : " << eulerAngle[0] <<", "<< eulerAngle[1] <<", "<< eulerAngle[2] <<"."<< endl;

        //对目标点云着色（红色）并可视化
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            target_color (target_cloud, 255, 0, 0);
        //对转换后的目标点云着色（绿色）并可视化
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            output_color (filtered_cloud, 0, 255, 0);

        mtx.lock();
        viewer.removeAllPointClouds();

        viewer.addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud0", v1);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            1, "target cloud0");
        viewer.addPointCloud<pcl::PointXYZ> (filtered_cloud, output_color, "output cloud0", v1);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            2, "output cloud0");
        viewer.addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud", v2);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            1, "target cloud");
        viewer.addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud", v2);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            2, "output cloud");

        mtx.unlock();

        target_cloud->clear();
        filtered_cloud->clear();

        return res;
    }

    void matching_Visualization()
    {
        while (ros::ok()&&!viewer.wasStopped())
        {
            mtx.lock();
            viewer.spinOnce(10);
            mtx.unlock();
        }
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

    void pcl_clear()
    {
        target_cloud->clear();
        output_cloud->clear();
        filtered_cloud->clear();
    }

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
        std::string fin = "/home/lsj/dev/Mine_WS/src/preception/include/Vertex.yaml";
        YAML::Node vertex_YN = YAML::LoadFile(fin);
        GraphLink map_Graph;
        vector<int> notExistIndex;
        for(YAML::const_iterator it=vertex_YN.begin();it!=vertex_YN.end();++it)
        {
            Intersection j;
            *it >> j;
            print(j);
            map_Graph.insert_Vertex(j.index, j.position,
                j.cluster_number, j.peak_number, j.search_radius);
            std::string fileName = filePath + std::to_string(j.index) + "_whole.pcd";
            if(access(fileName.c_str(), 0))
                notExistIndex.push_back(j.index);
        }
        if(!notExistIndex.empty())
        {
            ROS_ERROR("Cannot Find the Following Vertex Pcd !!!");
            for(const auto& i:notExistIndex)
                ROS_ERROR_STREAM("Vertex : " << i);
            exit(EXIT_FAILURE);
        }
        for(YAML::const_iterator it=vertex_YN.begin();it!=vertex_YN.end();++it)
        {
            Intersection j;
            *it >> j;
            for(const auto k : j.adjacent_index)
                map_Graph.insert_Edge(j.index, k);
        }
        fin = "/home/lsj/dev/Mine_WS/src/preception/include/Edge.yaml";
        YAML::Node Edge_YN = YAML::LoadFile(fin);
        for(YAML::const_iterator it=Edge_YN.begin();it!=Edge_YN.end();++it)
        {
            int index = (*it)["index"].as<int>();
            YAML::Node link_YN = (*it)["link"];
            for(YAML::const_iterator it_link=link_YN.begin();it_link!=link_YN.end();++it_link)
            {
                int link_index = (*it_link)["link_index"].as<int>();
                float begin_Angle = (*it_link)["begin_angle"].as<float>();
                float end_angle = (*it_link)["end_angle"].as<float>();
                map_Graph.insert_Edge(index, link_index, begin_Angle, end_angle);
            }
        }
        
        map_Graph.show_Vertex();
        map_Graph.show_Edge();
    /********读取拓扑地图信息，构建拓扑地图********/
    
    topoMap tpM;

    thread visual(&topoMap::matching_Visualization,&tpM);
    
    ros::Rate rate(1);
    while(ros::ok())
    {
        ros::spinOnce();

        // tpM.configLoad();

        tpM.vertex_Matching(map_Graph);

        tpM.pcl_clear();
        
        rate.sleep();
    }

    visual.join();


    return 0;
}