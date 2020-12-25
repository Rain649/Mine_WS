#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波
#include <pcl/filters/conditional_removal.h>    //条件滤波
#include <pcl/kdtree/kdtree_flann.h>    //kdtree搜索
#include <pcl/segmentation/extract_clusters.h>  //分割聚类
#include <cmath>
#include <vector>
#include <stdio.h>
#include <thread>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <preception/param_Config.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Float32MultiArray.h"
#include <pcl/visualization/pcl_plotter.h>
#include <visualization_msgs/Marker.h>

const int Horizon_SCAN = 360/2;

class Detection
{
private:
    // const int kernelSize = 5;
    const int N_SCAN = 16;
    // const float ang_res_x = 0.2;
    const float ang_res_y = 2.0;
    const float ang_bottom = 15.0;
    const int groundScanInd = 0;
    const int aboveScanInd = 16;
    const int colFL = 0;
    // const int colBL = 120;
    // const int colBR = 240;
    const int colFR = Horizon_SCAN;
    int xy_Condition;
    int width_Thre;
    int median_Size;
    int distance_Thre;
    int col_minus_Thre;
    int clusterRadius;
    int clusterSize_min;
    int index_Array[Horizon_SCAN];
    int Col_Array[10];
    int Cols_plot[10];
    double median_Coefficient;
    double Distance_plot[10];
    double timeLaserCloudNew;
    double Cols[Horizon_SCAN];
    double beam_Distance[Horizon_SCAN];

    bool receivePoints = false;
    bool over = true;
    bool outlier_Bool;
    bool medianFilter_Bool;

    std::vector<int> pointSearchInd;    //聚类索引
    std::vector<float> pointSearchSqDis;    //聚类距离
    std::vector<std::pair<int,double> > beam_Invalid;   //有效laser列索引、距离

    std_msgs::Bool intersectionDetected;
    std_msgs::Bool intersectionVerified;
    std_msgs::Int16 peak_Num;
    std_msgs::Int16 cluster_Num;

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloudNew;

    geometry_msgs::Point endPoint;
    
    visualization_msgs::Marker laser_Lane,Edge_Lane, circle_Lane;
    
    pcl::PointXYZI ExistPoint;
    pcl::PointXYZI nanPoint;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNew;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNewDS;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNewTFDS;
    pcl::PointCloud<pcl::PointXYZI>::Ptr straightLine;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudWithInfo;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFar;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudAfterCondition;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_3;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_4;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_5;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> cloud_after_StatisticalRemoval;  //统计滤波
    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_Condition;  //条件滤波
    pcl::ConditionalRemoval<pcl::PointXYZI> condition;
    pcl::visualization::PCLPlotter *plotter = new pcl::visualization::PCLPlotter("Dis_Ang");    //定义绘图器
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kd_first;

    ros::Publisher pubCloud;
    ros::Publisher pubCloudWithInfo;
    ros::Publisher pubCloudAfterCondition;
    ros::Publisher pubCloudFar;
    ros::Publisher pubIntersectionDetected;
    ros::Publisher pubIntersectionVerified;
    ros::Publisher pubBeamDistance;
    ros::Publisher pubLaserLane;
    ros::Publisher pubEdgeLane;
    ros::Publisher pubPeakNum;
    ros::Publisher pubClusterNum;
    ros::Publisher pubCluster_1;
    ros::Publisher pubCluster_2;
    ros::Publisher pubCluster_3;
    ros::Publisher pubCluster_4;
    ros::Publisher pubCluster_5;


public:
    Detection() : nh("~")
    {
        subLaserCloudNew = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_ma", 5, &Detection::laserCloudNewHandler, this);

        pubCloud = nh.advertise<sensor_msgs::PointCloud2>("Cloud", 5);
        pubCloudWithInfo = nh.advertise<sensor_msgs::PointCloud2>("cloudWithInfo", 10);
        pubCloudFar = nh.advertise<sensor_msgs::PointCloud2>("cloudFar", 5);
        pubCloudAfterCondition = nh.advertise<sensor_msgs::PointCloud2>("cloudAfterCondition", 5);
        pubIntersectionDetected = nh.advertise<std_msgs::Bool>("intersectionDetected", 5);
        pubIntersectionVerified = nh.advertise<std_msgs::Bool>("intersectionVerified", 5);
        pubBeamDistance = nh.advertise<std_msgs::Float32MultiArray>("beamDistance", 1);
        pubLaserLane = nh.advertise<visualization_msgs::Marker>("laserLane", 1);
        pubEdgeLane = nh.advertise<visualization_msgs::Marker>("edgeLane", 1);
        pubPeakNum = nh.advertise<std_msgs::Int16>("peakNum", 1);
        pubClusterNum = nh.advertise<std_msgs::Int16>("clusterNum", 1);
        pubCluster_1 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_1", 5);
        pubCluster_2 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_2", 5);
        pubCluster_3 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_3", 5);
        pubCluster_4 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_4", 5);
        pubCluster_5 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_5", 5);
        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = 0;

        intersectionDetected.data = false;
        intersectionVerified.data = false;

        downSizeFilter.setLeafSize(0.5, 0.5, 0.5);
  
        //设置特性
        plotter->setShowLegend(true);
        plotter->setXTitle("Beam");
        plotter->setYTitle("Distance");
        plotter->setXRange(0,Horizon_SCAN);

        for(int i=0;i<Horizon_SCAN;i++)
        {
            Cols[i] = i+1;
        }

        laser_Lane.header.frame_id = Edge_Lane.header.frame_id = circle_Lane.header.frame_id = "velodyne";
        laser_Lane.ns = Edge_Lane.ns = circle_Lane.ns = "intersection";
        laser_Lane.action = Edge_Lane.action = circle_Lane.action = visualization_msgs::Marker::ADD;
        laser_Lane.pose.orientation.w = Edge_Lane.pose.orientation.w = circle_Lane.pose.orientation.w = 1.0;
        laser_Lane.type = Edge_Lane.type = visualization_msgs::Marker::LINE_LIST;
        //laser_Lane
        laser_Lane.id = 0;
        laser_Lane.scale.x = 0.1;
        laser_Lane.color.b = 1.0;
        laser_Lane.color.a = 0.6;
        //Edge_Lane
        Edge_Lane.id = 1;
        Edge_Lane.scale.x = 0.2;
        Edge_Lane.color.g = 1.0;
        Edge_Lane.color.a = 1.0;
        // laser_Lane.lifetime = ros::Duration(100);
        //circle_Lane
        circle_Lane.type = visualization_msgs::Marker::LINE_STRIP;
        circle_Lane.id = 2;
        circle_Lane.scale.x = 0.1;
        circle_Lane.color.g = 1.0;
        circle_Lane.color.a = 1.0;

        allocateMemory();
    }

    void allocateMemory()
    {
        memset(beam_Distance,0,sizeof(beam_Distance));
        memset(index_Array,0,sizeof(index_Array));
        laserCloudNew.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laserCloudNewDS.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laserCloudNewTFDS.reset(new pcl::PointCloud<pcl::PointXYZI>());
        straightLine.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudWithInfo.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudFar.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudAfterCondition.reset(new pcl::PointCloud<pcl::PointXYZI>());
        range_Condition.reset(new pcl::ConditionOr<pcl::PointXYZI>());
        cloudCluster_1.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudCluster_2.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudCluster_3.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudCluster_4.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudCluster_5.reset(new pcl::PointCloud<pcl::PointXYZI>());
    
        //构建有序点云
        cloudWithInfo->points.resize(N_SCAN * Horizon_SCAN);
        std::fill(cloudWithInfo->points.begin(), cloudWithInfo->points.end(), nanPoint);
    }

    void clearMemory()
    {   
        memset(beam_Distance,0,sizeof(beam_Distance));
        memset(index_Array,0,sizeof(index_Array));
        peak_Num.data = 0;
        cluster_Num.data = 0;

        laserCloudNew->clear();
        laserCloudNewDS->clear();
        laserCloudNewTFDS->clear();
        straightLine->clear();
        std::fill(cloudWithInfo->points.begin(), cloudWithInfo->points.end(), nanPoint);
        cloudFar->clear();
        cloudAfterCondition->clear();
        plotter->clearPlots();
        laser_Lane.points.clear();
        Edge_Lane.points.clear();
        beam_Invalid.clear();
        // delete[] Cols_plot;
        // delete[] Distance_plot;

        over = true;
        receivePoints = false;
        intersectionDetected.data = false;
        intersectionVerified.data = false;

        cloudCluster_1->clear();
        cloudCluster_2->clear();
        cloudCluster_3->clear();
        cloudCluster_4->clear();
        cloudCluster_5->clear();

        range_Condition.reset(new pcl::ConditionOr<pcl::PointXYZI>());
    }

    void laserCloudNewHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        timeLaserCloudNew = msg->header.stamp.toSec();
        laser_Lane.header.stamp = msg->header.stamp;
        pcl::fromROSMsg(*msg, *laserCloudNew);

        downSizeFilter.setInputCloud(laserCloudNew);
        downSizeFilter.filter(*laserCloudNewDS);
        for (int k = 0; k < laserCloudNewDS->points.size(); k++)
        {
            ExistPoint = laserCloudNewDS->points[k];
            if (ExistPoint.intensity < 1)
                continue;
            // if (ExistPoint.z > 1.5 || ExistPoint.z < -2)
            //     continue;
            laserCloudNewTFDS->push_back(ExistPoint);
        }

        if(outlier_Bool)
        {
            //统计滤波
            cloud_after_StatisticalRemoval.setInputCloud(laserCloudNewTFDS);
            cloud_after_StatisticalRemoval.setMeanK(10);
            cloud_after_StatisticalRemoval.setStddevMulThresh(1.0);
            cloud_after_StatisticalRemoval.filter(*laserCloudNewTFDS);
        } 

        receivePoints = true;
    }

    void getCloudWithInfo()
    {
        float verticalAngle, horizonAngle;
        pcl::PointXYZI thisPoint;

        int num = 0;
        for (int i = 0; i < laserCloudNewTFDS->points.size(); i++)
        {
            int rowIdn, columnIdn, index;
            thisPoint.x = laserCloudNewTFDS->points[i].x;
            thisPoint.y = laserCloudNewTFDS->points[i].y;
            thisPoint.z = laserCloudNewTFDS->points[i].z;
            thisPoint.intensity = laserCloudNewTFDS->points[i].intensity;

            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y;

            // std::cout<<"rowIdn = "<< rowIdn << std::endl;

            if (rowIdn < groundScanInd || rowIdn >= aboveScanInd)
                continue;

            horizonAngle = atan2(thisPoint.y, thisPoint.x) * 180 / M_PI;
            if (thisPoint.y < 0)
                horizonAngle += 360;
            // std::cout<<"horizonAngle = "<< horizonAngle << std::endl;

            columnIdn = round(horizonAngle*Horizon_SCAN/360);
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            // if (columnIdn < colFL || columnIdn > colFR || columnIdn > colBL && columnIdn < colBR)
            //     continue;

            float range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);

            index = columnIdn + rowIdn * Horizon_SCAN;
            cloudWithInfo->points[index] = thisPoint;

            num += 1;
        }       
    }

    void laser_Visualizaton()
    {
        //求雷达每列最大距离
        int i=0;
        for(int columnIdn = 0; columnIdn < Horizon_SCAN; columnIdn++)
        {
            int max_Index = -1;
            for( int rowIdn = 0; rowIdn < N_SCAN; rowIdn++)
            {
                int index = columnIdn + rowIdn * Horizon_SCAN;
                double distance = sqrt(pow(cloudWithInfo->points[index].x,2) + pow(cloudWithInfo->points[index].y,2));
                if(beam_Distance[columnIdn] < distance)
                    {
                        beam_Distance[columnIdn] = distance;
                        max_Index = index;
                    }
            }
            if(max_Index == -1) 
            {
                continue;
            }
            index_Array[columnIdn] = max_Index;
            // Col_Array[i] = columnIdn;
            i += 1;
        }

        //中值（实际用为扩张）滤波
        if(median_Size%2==0) medianFilter_Bool = false;
        if(medianFilter_Bool)
        {
            double beam_Distance_0[Horizon_SCAN];
            for(int i=0;i<Horizon_SCAN;i++)
            {
                double M[median_Size];
                int k=0;
                for(int j=-median_Size/2;j<=median_Size/2;j++)
                {
                    int index = i+j;
                    if(index<0) index += Horizon_SCAN;
                    if(index>=Horizon_SCAN) index -= Horizon_SCAN;
                    M[k] = beam_Distance[index];
                    k += 1;
                }

                //排序
                for(int j=0;j<median_Size;j++)
                {
                    for(k=j+1;k<median_Size;k++)
                    {
                        if(M[j]>M[k])
                        {
                            double l = M[j];
                            M[j] = M[k];
                            M[k] = l;
                        }
                    }
                }
                beam_Distance_0[i] = M[(int)median_Coefficient*(median_Size-1)];
            }
            for(int i=0;i<Horizon_SCAN;i++)
            {
                beam_Distance[i] = beam_Distance_0[i];
            }
            // ROS_INFO("Median Filter");
        }

        //laser可视化处理
        for(int columnIdn = 0; columnIdn < Horizon_SCAN; columnIdn++)
        {
            if(index_Array[columnIdn] == 0) continue;
            endPoint.x = endPoint.y = endPoint.z = 0;
            laser_Lane.points.push_back(endPoint);

            endPoint.x = cloudWithInfo->points[index_Array[columnIdn]].x;
            endPoint.y = cloudWithInfo->points[index_Array[columnIdn]].y;
            endPoint.z = cloudWithInfo->points[index_Array[columnIdn]].z;
            laser_Lane.points.push_back(endPoint);
            beam_Invalid.push_back(std::pair<int,double>(columnIdn,beam_Distance[columnIdn]));
        }

        //距离-角度分布图
        plotter->addPlotData(Cols, beam_Distance, Horizon_SCAN, "直方图");
        plotter->spinOnce(0);
             
        //距离阈值可视化           
        circle_Lane.points.clear();
        circle_Lane.header.stamp = laser_Lane.header.stamp;
        for(int i=0;i<360;i++)
        {
            geometry_msgs::Point circle;
            circle.x = distance_Thre*cos(i/2/M_PI);
            circle.y = distance_Thre*sin(i/2/M_PI);
            circle.z = 0;
            circle_Lane.points.push_back(circle);
        }
    }

    void intersectionDetection()
    {
        int rowIdn, columnIdn;
        Edge_Lane.header.stamp = laser_Lane.header.stamp;

        //横向距离阈值判定路口
        for (rowIdn = groundScanInd; rowIdn < aboveScanInd; rowIdn++)
        {
            for (columnIdn = colFL; columnIdn < colFR; columnIdn++)
            {
                pcl::PointXYZI thisPoint;
                int index = columnIdn + rowIdn * Horizon_SCAN;
                thisPoint = cloudWithInfo->points[index];

                if (abs(thisPoint.y) > 5 && abs(thisPoint.x) <= 20)
                {
                    cloudFar->push_back(thisPoint);
                }
            }
        }

        //从vector提取数组，方便后续使用
        int* Cols_plot = new int[beam_Invalid.size()];
        double* Distance_plot = new double[beam_Invalid.size()];
        int i_plot = 0;
        for(std::vector<std::pair<int,double> >::iterator itr=beam_Invalid.begin();itr!=beam_Invalid.end();itr++)
        {
            Cols_plot[i_plot] = itr->first;
            // std::cout << itr->first << std::endl;
            Distance_plot[i_plot] = itr->second;
            // std::cout << itr->second << std::endl;
            i_plot += 1;
        }

        //路口数量判定
        int numOfEdgeLaser = 0;
        for(int i=0;i<beam_Invalid.size();i++)
        {
            //跳过最初相位的波峰
            if(i==0 && Distance_plot[i]>distance_Thre)
            {
                while(Distance_plot[i]>distance_Thre)
                {
                    i += 1;                        
                }
            }

            int i_neighbor = i+1;
            if(i_neighbor >= beam_Invalid.size()) i_neighbor -= beam_Invalid.size();

            //满足边界判断条件，选定左边界
            if(Distance_plot[i_neighbor]>distance_Thre  && Distance_plot[i_neighbor]>Distance_plot[i])
            {
                int i_right = i_neighbor + 1;
                if(i_right>=beam_Invalid.size()) i_right -= beam_Invalid.size();
                int col_Minus = Cols_plot[i_right]-Cols_plot[i];
                if(col_Minus<0) col_Minus += Horizon_SCAN;
                while(col_Minus<=col_minus_Thre*Horizon_SCAN/360)
                {
                    // i_right += (i_right==beam_Invalid.size()-1) ? 1-beam_Invalid.size() : 1;
                    if(Distance_plot[i_right]<distance_Thre)
                    {
                        ROS_DEBUG_STREAM("i =  =  " << i);
                        ROS_DEBUG_STREAM("i_right =  =  " << i_right);

                        //若满足右边界判断条件，进行可视化处理
                        if((Distance_plot[i_right]+Distance_plot[i])*sin(col_Minus*M_PI/Horizon_SCAN)>width_Thre)
                        // if(pow(cloudWithInfo->points[index_Array[Cols_plot[i_right]]].x - 
                        //     cloudWithInfo->points[index_Array[Cols_plot[i]]].x,2) + 
                        //     pow(cloudWithInfo->points[index_Array[Cols_plot[i_right]]].y - 
                        //     cloudWithInfo->points[index_Array[Cols_plot[i]]].y,2) > pow(width_Thre,2))
                        {
                            if(index_Array[Cols_plot[i]])
                            {
                                //mark this beam
                                endPoint.x = endPoint.y = endPoint.z = 0;
                                Edge_Lane.points.push_back(endPoint);

                                endPoint.x = cloudWithInfo->points[index_Array[Cols_plot[i]]].x;
                                endPoint.y = cloudWithInfo->points[index_Array[Cols_plot[i]]].y;
                                endPoint.z = cloudWithInfo->points[index_Array[Cols_plot[i]]].z;
                                Edge_Lane.points.push_back(endPoint);

                                numOfEdgeLaser += 1;
                            }

                            if(index_Array[Cols_plot[i_right]])
                            {
                                //mark right beam
                                endPoint.x = endPoint.y = endPoint.z = 0;
                                Edge_Lane.points.push_back(endPoint);

                                endPoint.x = cloudWithInfo->points[index_Array[Cols_plot[i_right]]].x;
                                endPoint.y = cloudWithInfo->points[index_Array[Cols_plot[i_right]]].y;
                                endPoint.z = cloudWithInfo->points[index_Array[Cols_plot[i_right]]].z;
                                Edge_Lane.points.push_back(endPoint);

                                numOfEdgeLaser += 1;
                            }
                            
                            if(index_Array[Cols_plot[i_neighbor]])
                            {
                                //mark this beam
                                endPoint.x = endPoint.y = endPoint.z = 0;
                                Edge_Lane.points.push_back(endPoint);

                                endPoint.x = cloudWithInfo->points[index_Array[Cols_plot[i_neighbor]]].x;
                                endPoint.y = cloudWithInfo->points[index_Array[Cols_plot[i_neighbor]]].y;
                                endPoint.z = cloudWithInfo->points[index_Array[Cols_plot[i_neighbor]]].z;
                                Edge_Lane.points.push_back(endPoint);

                                numOfEdgeLaser += 1;
                            }

                            int i_right_neighbor = i_right - 1;
                            if(i_right_neighbor<0) i_right_neighbor = beam_Invalid.size()-1;
                            if(index_Array[Cols_plot[i_right_neighbor]])
                            {
                                //mark right beam
                                endPoint.x = endPoint.y = endPoint.z = 0;
                                Edge_Lane.points.push_back(endPoint);

                                endPoint.x = cloudWithInfo->points[index_Array[Cols_plot[i_right_neighbor]]].x;
                                endPoint.y = cloudWithInfo->points[index_Array[Cols_plot[i_right_neighbor]]].y;
                                endPoint.z = cloudWithInfo->points[index_Array[Cols_plot[i_right_neighbor]]].z;
                                Edge_Lane.points.push_back(endPoint);

                                numOfEdgeLaser += 1;
                            }

                            ROS_DEBUG_STREAM("i =   " << i);
                            ROS_DEBUG_STREAM("i_right =   " << i_right);
                            ROS_DEBUG_STREAM("col minus =   " << col_Minus);

                            //统计路口数量
                            peak_Num.data += 1;

                            //完成一个周期，跳出本次循环
                            if(i<i_right) i = i_right;
                                else goto part1;
                            break;
                        }
                    }

                    //右边界索引迭代，计算右边界与左边界水平索引差值
                    i_right += 1;
                    if(i_right==beam_Invalid.size()) i_right = 0;
                    col_Minus = Cols_plot[i_right]-Cols_plot[i];
                    if(col_Minus<0) col_Minus += Horizon_SCAN;
                }
            }
        }

        //跳到此处
        part1:
            ROS_DEBUG("Jump to Part 1");
            
        //laser可视数量
        ROS_DEBUG_STREAM("Num of Edge Laser =   " << numOfEdgeLaser);

        if (peak_Num.data > 2)
        {
            intersectionDetected.data = true;
            std::cout << "Intersection Number = " << peak_Num.data << std::endl;
            std::cout << "------------------------------------------------------" << std::endl;
            std::cout << "****检测到洞口****************" << std::endl;
        }

        if (cloudFar->points.size() >= 30)
            std::cout << "                    Size of Far >= " << cloudFar->points.size() << std::endl;
        if (cloudFar->points.size() >= 40)
        {
            intersectionVerified.data = true;
            std::cout << "------------------------------------------------------" << std::endl;
            std::cout << "******************确认到达洞口" << std::endl;
        }
    }

    void intersectionDivide()
    {
        //条件设定
        range_Condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new
            pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GE, xy_Condition)));  //GT表示大于等于
        range_Condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new
            pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LE, -xy_Condition)));  //GT表示大于等于
        range_Condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new
            pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GE, xy_Condition)));  //LT表示小于等于
        range_Condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new
            pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LE, -xy_Condition)));  //LT表示小于等于

        //条件滤波
        condition.setCondition(range_Condition);
        condition.setInputCloud(laserCloudNew);
        condition.setKeepOrganized(false);
        condition.filter(*cloudAfterCondition);
    }

    // void set_Condition(pcl::PointXYZI>::Ptr msg)
    // {
    //         int x = msg->points.x;
    //         int y = msg->points.y;
    //         int dis = pow(x,2) + pow(y,2);
    //         if(dis>pow(xy_Condition,2)) return true;
    //         else return false;
    // }

    void intersectionCluster()
    {
        // if(cloudAfterCondition->width == 0) return;

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloudAfterCondition); //创建点云索引向量，用于存储实际的点云信息

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(clusterRadius); //设置近邻搜索的搜索半径
        ec.setMinClusterSize(clusterSize_min);    //设置一个聚类需要的最少点数目
        // ec.setMaxClusterSize(1000);   //设置一个聚类需要的最大点数目
        ec.setSearchMethod(tree);    //设置点云的搜索机制
        ec.setInputCloud(cloudAfterCondition);
        ec.extract(cluster_indices); //从点云中提取聚类，并将点云索引保存在cluster_indices中

        /*为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中。*/
        //迭代访问点云索引cluster_indices，直到分割出所有聚类
        cluster_Num.data = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            cluster_Num.data++;

            switch (cluster_Num.data)
            {
            case 1:
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloudCluster_1->points.push_back(cloudAfterCondition->points[*pit]);
                cloudCluster_1->width = cloudCluster_1->points.size();
                cloudCluster_1->height = 1;
                cloudCluster_1->is_dense = true;

                break;

            case 2:
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloudCluster_2->points.push_back(cloudAfterCondition->points[*pit]);
                cloudCluster_2->width = cloudCluster_2->points.size();
                cloudCluster_2->height = 1;
                cloudCluster_2->is_dense = true;
                break;

            case 3:
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloudCluster_3->points.push_back(cloudAfterCondition->points[*pit]);
                cloudCluster_3->width = cloudCluster_3->points.size();
                cloudCluster_3->height = 1;
                cloudCluster_3->is_dense = true;

                break;

            case 4:
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloudCluster_4->points.push_back(cloudAfterCondition->points[*pit]);
                cloudCluster_4->width = cloudCluster_4->points.size();
                cloudCluster_4->height = 1;
                cloudCluster_4->is_dense = true;

                break;

            case 5:
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloudCluster_5->points.push_back(cloudAfterCondition->points[*pit]);
                cloudCluster_5->width = cloudCluster_5->points.size();
                cloudCluster_5->height = 1;
                cloudCluster_5->is_dense = true;

                break;

            default:
                break;
            }
        }
    }

    void publishResult()
    {
        if (pubCloud.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*laserCloudNewTFDS, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCloud.publish(cloudMsgTemp);
        }

        if (pubCloudWithInfo.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudWithInfo, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCloudWithInfo.publish(cloudMsgTemp);
        }

        if (pubCloudFar.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudFar, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCloudFar.publish(cloudMsgTemp);
        }

        if (pubCloudAfterCondition.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudAfterCondition, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCloudAfterCondition.publish(cloudMsgTemp);
        }

        if (pubCluster_1.getNumSubscribers() != 0 )
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_1, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCluster_1.publish(cloudMsgTemp);
        }

        if (pubCluster_2.getNumSubscribers() != 0 )
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_2, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCluster_2.publish(cloudMsgTemp);
        }

        if (pubCluster_3.getNumSubscribers() != 0 )
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_3, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCluster_3.publish(cloudMsgTemp);
        }

        if (pubCluster_4.getNumSubscribers() != 0 )
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_4, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCluster_4.publish(cloudMsgTemp);
        }

        if (pubCluster_5.getNumSubscribers() != 0 )
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_5, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCluster_5.publish(cloudMsgTemp);
        }

        if (pubIntersectionDetected.getNumSubscribers() != 0)
        {
            pubIntersectionDetected.publish(intersectionDetected);
        }

        if (pubIntersectionVerified.getNumSubscribers() != 0)
        {
            pubIntersectionVerified.publish(intersectionVerified);
        }

        if (1)
        {
            pubLaserLane.publish(laser_Lane);
            pubEdgeLane.publish(Edge_Lane);
            pubEdgeLane.publish(circle_Lane);
            pubPeakNum.publish(peak_Num);
            pubClusterNum.publish(cluster_Num);
            
            std_msgs::Float32MultiArray beam_Dis;
            for (int i = 0; i < Horizon_SCAN; i++)
            {
                beam_Dis.data.push_back(beam_Distance[i]);
            }
            pubBeamDistance.publish(beam_Dis);
        }
    }

    void run()
    {
        if (receivePoints && over)
        {
            over = false;
            getDynamicParameter();
            getCloudWithInfo();
            laser_Visualizaton();
            intersectionDetection();
            if(intersectionVerified.data || true)
            {
                loop:
                    ROS_INFO_STREAM("Condition  ===========    " << xy_Condition);
                    intersectionDivide();
                    intersectionCluster();
                    
                if(cluster_Num.data<peak_Num.data && xy_Condition<16)
                {   
                    xy_Condition += 2;
                    range_Condition.reset(new pcl::ConditionOr<pcl::PointXYZI>());
                    cloudAfterCondition->clear();
                    cloudCluster_1->clear();
                    cloudCluster_2->clear();
                    cloudCluster_3->clear();
                    cloudCluster_4->clear();
                    cloudCluster_5->clear();
                    goto loop;
                }
            }
            publishResult();
            clearMemory();
        }
    }

    void getDynamicParameter()
    {
        ros::param::get("/intersection/xy_condition",xy_Condition);
        ros::param::get("/intersection/width_threshold",width_Thre);
        ros::param::get("/intersection/distance_threshold",distance_Thre);
        ros::param::get("/intersection/col_minus_threshold",col_minus_Thre);
        ros::param::get("/intersection/cluster_radius",clusterRadius);
        ros::param::get("/intersection/cluster_size_min",clusterSize_min);
        ros::param::get("/intersection/bool_outlier_removal",outlier_Bool);
        ros::param::get("/intersection/bool_median_filter",medianFilter_Bool);
        ros::param::get("/intersection/median_size",median_Size);
        ros::param::get("/intersection/median_coefficient",median_Coefficient);
    }
};

//动态调参
void callback(preception::param_Config &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %d %d %d %d %d %d %d %f %s %s %s %d",
             config.xy_condition,
             config.width_threshold,
             config.distance_threshold,
             config.col_minus_threshold,
             config.median_size,
             config.cluster_radius,
             config.cluster_size_min,
             config.median_coefficient,
             config.str_param.c_str(),
             config.bool_outlier_removal ? "True" : "False",
             config.bool_median_filter ? "True" : "False",
             config.size);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "intersection");

    //动态参数调节
    dynamic_reconfigure::Server<preception::param_Config> server;
    dynamic_reconfigure::Server<preception::param_Config>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("\033[1;32m---->\033[0m Intersection Detection Started.");

    Detection ND;

    ros::Rate rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        ND.run();
        rate.sleep();
    }

    return 0;
}
