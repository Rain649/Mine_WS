
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
#include <pcl/kdtree/kdtree_flann.h>                 //kdtree搜索
#include <pcl/segmentation/extract_clusters.h>       //分割聚类
#include <pcl/filters/extract_indices.h>             //按索引提取
#include <pcl/filters/passthrough.h>                 //直通滤波
#include <pcl/filters/conditional_removal.h>         //条件滤波
#include <cmath>
#include <vector>
#include <stdio.h>
#include <thread>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <perception/param_Config.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/visualization/pcl_plotter.h>
#include <visualization_msgs/Marker.h>
#include <utility.h>
#include <opencv2/opencv.hpp>
#include <numeric>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

class Detection
{
private:
    int segmentationRadius;
    int width_Thre;
    int median_Size;
    int distance_Thre;
    int col_minus_Thre;
    int clusterSize_min;
    int index_Array[Horizon_SCAN];
    double clusterRadius;
    double median_Coefficient;
    double timeLaserCloudNew;
    double Cols[Horizon_SCAN];
    double beamDistance_Vec[Horizon_SCAN];

    bool receivePoints = false;
    bool over = true;
    bool outlier_Bool;
    bool medianFilter_Bool;

    std::vector<std::pair<int, double>> beam_Invalid; //有效laser列索引、距离

    std_msgs::Bool intersectionDetected;
    std_msgs::Bool intersectionVerified;
    std_msgs::UInt8 peak_Num;
    std_msgs::UInt8 cluster_Num;
    std_msgs::Float32 peakDistance_Max;

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloudNew;

    geometry_msgs::Point endPoint;

    visualization_msgs::Marker laser_Lane, Edge_Lane, circle_Lane;

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform fixedTrans;

    pcl::PointXYZI nanPoint;
    pcl::PointXYZI egoPoint;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserOrigin_Cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr outlierRemove_Cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNewTFDS;
    pcl::PointCloud<pcl::PointXYZI>::Ptr straightLine;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudWithInfo;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFar;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIntersections;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_3;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_4;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_5;
    // pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> cloud_after_StatisticalRemoval; //统计滤波
    pcl::ConditionAnd<pcl::PointXYZI>::Ptr longitudinal_Condition;                 //条件滤波
    pcl::ConditionOr<pcl::PointXYZI>::Ptr lateral_Condition;                       //条件滤波
    pcl::ConditionalRemoval<pcl::PointXYZI> condition;
    pcl::visualization::PCLPlotter *plotter = new pcl::visualization::PCLPlotter("Dis_Ang"); //定义绘图器
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kd_first;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    pcl::ExtractIndices<pcl::PointXYZI> extract;

    ros::Publisher pubCloudOutlierRemove;
    ros::Publisher pubCloudWithInfo;
    ros::Publisher pubCloudIntersection;
    ros::Publisher pubCloudFar;
    ros::Publisher pubIntersectionDetected;
    ros::Publisher pubIntersectionVerified;
    ros::Publisher pubBeamDistance;
    ros::Publisher pubLaserLane;
    ros::Publisher pubEdgeLane;
    ros::Publisher peakDistanceMax;
    ros::Publisher pubPeakNum;
    ros::Publisher pubClusterNum;
    ros::Publisher pubSegmentationRadius;
    ros::Publisher pubCluster_1;
    ros::Publisher pubCluster_2;
    ros::Publisher pubCluster_3;
    ros::Publisher pubCluster_4;
    ros::Publisher pubCluster_5;

public:
    Detection() : nh("~")
    {
        subLaserCloudNew = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_ma", 1, &Detection::laserCloudNewHandler, this);

        pubCloudOutlierRemove = nh.advertise<sensor_msgs::PointCloud2>("cloud_outlier_remove", 1);
        pubCloudWithInfo = nh.advertise<sensor_msgs::PointCloud2>("cloudWithInfo", 1);
        pubCloudFar = nh.advertise<sensor_msgs::PointCloud2>("cloudFar", 1);
        pubCloudIntersection = nh.advertise<sensor_msgs::PointCloud2>("cloudIntersections", 1);
        pubIntersectionDetected = nh.advertise<std_msgs::Bool>("intersectionDetected", 1);
        pubIntersectionVerified = nh.advertise<std_msgs::Bool>("intersectionVerified", 1);
        pubBeamDistance = nh.advertise<std_msgs::Float32MultiArray>("beamDistance", 1);
        pubLaserLane = nh.advertise<visualization_msgs::Marker>("laserLane", 1);
        pubEdgeLane = nh.advertise<visualization_msgs::Marker>("edgeLane", 1);
        pubPeakNum = nh.advertise<std_msgs::UInt8>("peakNum", 1);
        pubClusterNum = nh.advertise<std_msgs::UInt8>("clusterNum", 1);
        peakDistanceMax = nh.advertise<std_msgs::Float32>("peakDistance_Max", 1);
        pubSegmentationRadius = nh.advertise<std_msgs::UInt8>("segmentationRadius", 1);
        pubCluster_1 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_1", 1);
        pubCluster_2 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_2", 1);
        pubCluster_3 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_3", 1);
        pubCluster_4 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_4", 1);
        pubCluster_5 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_5", 1);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        intersectionDetected.data = false;
        intersectionVerified.data = false;

        // downSizeFilter.setLeafSize(0.5, 0.5, 0.5);

        //设置特性
        plotter->setShowLegend(true);
        plotter->setXTitle("Beam");
        plotter->setYTitle("Distance");
        plotter->setXRange(0, Horizon_SCAN);

        for (int i = 0; i < Horizon_SCAN; ++i)
        {
            Cols[i] = i;
        }

        laser_Lane.header.frame_id = Edge_Lane.header.frame_id = circle_Lane.header.frame_id = "base_link";
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
        //circle_Lane
        circle_Lane.type = visualization_msgs::Marker::LINE_STRIP;
        circle_Lane.id = 2;
        circle_Lane.scale.x = 0.1;
        circle_Lane.color.g = 1.0;
        circle_Lane.color.a = 1.0;

        egoPoint.x = egoPoint.y = egoPoint.z = egoPoint.intensity = 0;

        allocateMemory();
    }

    void allocateMemory()
    {
        memset(beamDistance_Vec, 0, sizeof(beamDistance_Vec));
        memset(index_Array, 0, sizeof(index_Array));
        laserOrigin_Cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        outlierRemove_Cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laserCloudNewTFDS.reset(new pcl::PointCloud<pcl::PointXYZI>());
        straightLine.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudWithInfo.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudFar.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudIntersections.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudCluster_1.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudCluster_2.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudCluster_3.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudCluster_4.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudCluster_5.reset(new pcl::PointCloud<pcl::PointXYZI>());
        longitudinal_Condition.reset(new pcl::ConditionAnd<pcl::PointXYZI>());
        lateral_Condition.reset(new pcl::ConditionOr<pcl::PointXYZI>());

        fixedTrans.frame_id_ = "/velodyne";
        fixedTrans.child_frame_id_ = "/base_link";
        fixedTrans.setRotation(tf::Quaternion(0, 0, 0, 1));
        fixedTrans.setOrigin(tf::Vector3(0, 0, 0));

        ROS_DEBUG("Allocate Memory Success !!!");
    }

    void clearMemory()
    {
        memset(beamDistance_Vec, 0, sizeof(beamDistance_Vec));
        memset(index_Array, 0, sizeof(index_Array));
        peak_Num.data = 0;
        cluster_Num.data = 0;

        laserOrigin_Cloud->clear();
        outlierRemove_Cloud->clear();
        laserCloudNewTFDS->clear();
        straightLine->clear();
        cloudFar->clear();
        cloudIntersections->clear();
        plotter->clearPlots();
        laser_Lane.points.clear();
        Edge_Lane.points.clear();
        beam_Invalid.clear();

        over = true;
        receivePoints = false;
        intersectionDetected.data = false;
        intersectionVerified.data = false;

        cloudCluster_1->clear();
        cloudCluster_2->clear();
        cloudCluster_3->clear();
        cloudCluster_4->clear();
        cloudCluster_5->clear();

        ROS_DEBUG("Clear Memory Success !!!");
    }

    void laserCloudNewHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        timeLaserCloudNew = msg->header.stamp.toSec();
        laser_Lane.header.stamp = msg->header.stamp;
        pcl::fromROSMsg(*msg, *laserOrigin_Cloud);

        if (outlier_Bool)
        {
            //统计滤波
            cloud_after_StatisticalRemoval.setInputCloud(laserOrigin_Cloud);
            cloud_after_StatisticalRemoval.setMeanK(10);
            cloud_after_StatisticalRemoval.setStddevMulThresh(2.0);
            cloud_after_StatisticalRemoval.filter(*outlierRemove_Cloud);
        }

        receivePoints = true;
        ROS_DEBUG("laserCloudNewHandler Success !!!");
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr getCloudWithInfo(pcl::PointCloud<pcl::PointXYZI>::Ptr input_Cloud, const int row_Min, const int row_Max, const int horizon_Num)
    {
        /*ang_bottom和ang_res_y是全局变量,由外部定义*/

        float verticalAngle, horizonAngle;
        pcl::PointXYZI thisPoint;
        pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>());
        //构建有序点云
        result->points.resize(N_SCAN * Horizon_SCAN);
        std::fill(result->points.begin(), result->points.end(), nanPoint);

        for (int i = 0; i < input_Cloud->points.size(); i++)
        {
            int rowIdn, columnIdn, index;
            thisPoint.x = input_Cloud->points[i].x;
            thisPoint.y = input_Cloud->points[i].y;
            thisPoint.z = input_Cloud->points[i].z;
            thisPoint.intensity = input_Cloud->points[i].intensity;

            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y;

            if (rowIdn < row_Min || rowIdn >= row_Max)
                continue;

            horizonAngle = atan2(thisPoint.y, thisPoint.x) * 180 / M_PI;
            if (thisPoint.y < 0)
                horizonAngle += 360;

            columnIdn = round(horizonAngle * horizon_Num / 360);
            if (columnIdn >= horizon_Num)
                columnIdn -= horizon_Num;

            float range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);

            index = columnIdn + rowIdn * horizon_Num;
            result->points[index] = thisPoint;
        }

        ROS_DEBUG("Get Cloud With Info Success !!!");
        return result;
    }

    void laser_Visualizaton()
    {
        /*求雷达每列最大距离*/
        for (int columnIdn = 0; columnIdn < Horizon_SCAN; ++columnIdn)
        {
            int max_Index = -1;
            for (int rowIdn = 0; rowIdn < N_SCAN; ++rowIdn)
            {
                int index = columnIdn + rowIdn * Horizon_SCAN;
                double distance = sqrt(pow(cloudWithInfo->points[index].x, 2) + pow(cloudWithInfo->points[index].y, 2));
                if (beamDistance_Vec[columnIdn] < distance)
                {
                    beamDistance_Vec[columnIdn] = distance;
                    max_Index = index;
                }
            }
            if (max_Index == -1)
                continue;
            index_Array[columnIdn] = max_Index;
        }
        /*求雷达每列最大距离*/

        /*中值滤波（实际用为扩张）*/
        if (medianFilter_Bool)
        {
            double beam_Distance_0[Horizon_SCAN];
            for (int i = 0; i < Horizon_SCAN; ++i)
            {
                std::vector<double> beamNeighbor_Vec;
                int k = 0;
                for (int j = -median_Size; j <= median_Size; ++j)
                {
                    int index = i + j;
                    if (index < 0)
                        index += Horizon_SCAN;
                    if (index >= Horizon_SCAN)
                        index -= Horizon_SCAN;
                    beamNeighbor_Vec.push_back(beamDistance_Vec[index]);
                }

                //排序
                sort(beamNeighbor_Vec.begin(), beamNeighbor_Vec.end());

                beam_Distance_0[i] = beamNeighbor_Vec[(int)median_Coefficient * (median_Size - 1)];
            }

            for (int i = 0; i < Horizon_SCAN; ++i)
            {
                beamDistance_Vec[i] = beam_Distance_0[i];
            }

            ROS_DEBUG("Median Filter Success !!!");
        }
        /*中值滤波（实际用为扩张）*/

        /*laser可视化处理*/
        for (int columnIdn = 0; columnIdn < Horizon_SCAN; ++columnIdn)
        {
            if (index_Array[columnIdn] == 0)
                continue;
            endPoint.x = endPoint.y = endPoint.z = 0;
            laser_Lane.points.push_back(endPoint);

            endPoint.x = cloudWithInfo->points[index_Array[columnIdn]].x;
            endPoint.y = cloudWithInfo->points[index_Array[columnIdn]].y;
            endPoint.z = cloudWithInfo->points[index_Array[columnIdn]].z;
            laser_Lane.points.push_back(endPoint);
            beam_Invalid.push_back(std::pair<int, double>(columnIdn, beamDistance_Vec[columnIdn]));
        }
        /*laser可视化处理*/

        //beam model(距离-角度分布图)
        plotter->addPlotData(Cols, beamDistance_Vec, Horizon_SCAN, "直方图");
        plotter->spinOnce(0);

        //距离阈值可视化
        circle_Lane.points.clear();
        circle_Lane.header.stamp = laser_Lane.header.stamp;
        for (int i = 0; i < 360; ++i)
        {
            geometry_msgs::Point circle;
            circle.x = distance_Thre * cos(i / 2 / M_PI);
            circle.y = distance_Thre * sin(i / 2 / M_PI);
            circle.z = 0;
            circle_Lane.points.push_back(circle);
        }

        ROS_DEBUG("Laser Visualizaton Success !!!");
    }

    void intersectionDetection()
    {
        int rowIdn, columnIdn;
        int sizeOfBeamInvalid = beam_Invalid.size();
        std::vector<double> peakPoints_Vec;
        std::vector<double> peakDistance_Vec;
        std::vector<std::pair<int, int>> startEnd_Vec;
        Edge_Lane.header.stamp = laser_Lane.header.stamp;

        /*横向距离阈值判定路口*/
        longitudinal_Condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GE, -5))); //GT表示大于等于
        longitudinal_Condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LE, 5)));  //GT表示大于等于

        lateral_Condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GE, 5)));  //LT表示小于等于
        lateral_Condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LE, -5))); //LT表示小于等于        //条件滤波

        condition.setCondition(longitudinal_Condition);
        condition.setInputCloud(laserOrigin_Cloud);
        condition.setKeepOrganized(false);
        condition.filter(*cloudFar);

        condition.setCondition(lateral_Condition);
        condition.setInputCloud(cloudFar);
        condition.setKeepOrganized(false);
        condition.filter(*cloudFar);
        /*横向距离阈值判定路口*/

        /*从vector提取数组，方便后续使用*/
        int Cols_plot[sizeOfBeamInvalid];
        double Distance_plot[sizeOfBeamInvalid];
        int i_plot = 0;
        for (std::vector<std::pair<int, double>>::iterator itr = beam_Invalid.begin(); itr != beam_Invalid.end(); ++itr)
        {
            Cols_plot[i_plot] = itr->first;
            Distance_plot[i_plot++] = itr->second;
        }
        /*从vector提取数组，方便后续使用*/

        /*路口数量检测*/
        int numOfEdgeLaser = 0;
        for (int i_left = 0; i_left < sizeOfBeamInvalid; ++i_left)
        {
            //跳过最初相位的波峰
            if (i_left == 0)
                while (Distance_plot[i_left] > distance_Thre)
                    ++i_left;

            int i_neighbor = i_left + 1;
            if (i_neighbor >= sizeOfBeamInvalid)
                i_neighbor -= sizeOfBeamInvalid;

            //满足边界判断条件，选定左边界
            if (Distance_plot[i_neighbor] > distance_Thre && Distance_plot[i_neighbor] > Distance_plot[i_left])
            {
                int i_right = i_neighbor + 1;
                if (i_right >= sizeOfBeamInvalid)
                    i_right -= sizeOfBeamInvalid;
                int colIndexMinus = Cols_plot[i_right] - Cols_plot[i_left];
                if (colIndexMinus < 0)
                    colIndexMinus += Horizon_SCAN;
                while (colIndexMinus <= col_minus_Thre * Horizon_SCAN / 360)
                {
                    if (Distance_plot[i_right] < distance_Thre)
                    {
                        ROS_DEBUG_STREAM("i_left =  =  " << i_left);
                        ROS_DEBUG_STREAM("i_right =  =  " << i_right);

                        //若满足右边界判断条件，进行可视化处理
                        if ((Distance_plot[i_right] + Distance_plot[i_left]) * sin(colIndexMinus * M_PI / Horizon_SCAN) > width_Thre)
                        {
                            startEnd_Vec.push_back(std::pair<int, int>(i_left, i_right));

                            if (index_Array[Cols_plot[i_left]])
                            {
                                //mark this beam
                                endPoint.x = endPoint.y = endPoint.z = 0;
                                Edge_Lane.points.push_back(endPoint);

                                endPoint.x = cloudWithInfo->points[index_Array[Cols_plot[i_left]]].x;
                                endPoint.y = cloudWithInfo->points[index_Array[Cols_plot[i_left]]].y;
                                endPoint.z = cloudWithInfo->points[index_Array[Cols_plot[i_left]]].z;
                                Edge_Lane.points.push_back(endPoint);

                                numOfEdgeLaser++;
                            }

                            if (index_Array[Cols_plot[i_right]])
                            {
                                //mark right beam
                                endPoint.x = endPoint.y = endPoint.z = 0;
                                Edge_Lane.points.push_back(endPoint);

                                endPoint.x = cloudWithInfo->points[index_Array[Cols_plot[i_right]]].x;
                                endPoint.y = cloudWithInfo->points[index_Array[Cols_plot[i_right]]].y;
                                endPoint.z = cloudWithInfo->points[index_Array[Cols_plot[i_right]]].z;
                                Edge_Lane.points.push_back(endPoint);

                                ++numOfEdgeLaser;
                            }

                            if (index_Array[Cols_plot[i_neighbor]])
                            {
                                //mark this beam
                                endPoint.x = endPoint.y = endPoint.z = 0;
                                Edge_Lane.points.push_back(endPoint);

                                endPoint.x = cloudWithInfo->points[index_Array[Cols_plot[i_neighbor]]].x;
                                endPoint.y = cloudWithInfo->points[index_Array[Cols_plot[i_neighbor]]].y;
                                endPoint.z = cloudWithInfo->points[index_Array[Cols_plot[i_neighbor]]].z;
                                Edge_Lane.points.push_back(endPoint);

                                numOfEdgeLaser++;
                            }

                            int i_right_neighbor = i_right - 1;
                            if (i_right_neighbor < 0)
                                i_right_neighbor = sizeOfBeamInvalid - 1;
                            if (index_Array[Cols_plot[i_right_neighbor]])
                            {
                                //mark right beam
                                endPoint.x = endPoint.y = endPoint.z = 0;
                                Edge_Lane.points.push_back(endPoint);

                                endPoint.x = cloudWithInfo->points[index_Array[Cols_plot[i_right_neighbor]]].x;
                                endPoint.y = cloudWithInfo->points[index_Array[Cols_plot[i_right_neighbor]]].y;
                                endPoint.z = cloudWithInfo->points[index_Array[Cols_plot[i_right_neighbor]]].z;
                                Edge_Lane.points.push_back(endPoint);

                                ++numOfEdgeLaser;
                            }

                            //统计路口数量
                            ++peak_Num.data;

                            //完成一个周期，跳出本次循环
                            if (i_left < i_right)
                                i_left = i_right;
                            else
                                goto part1;
                            break;
                        }
                    }

                    //右边界索引迭代，计算右边界与左边界水平索引差值
                    ++i_right;
                    if (i_right == sizeOfBeamInvalid)
                        i_right = 0;
                    colIndexMinus = Cols_plot[i_right] - Cols_plot[i_left];
                    if (colIndexMinus < 0)
                        colIndexMinus += Horizon_SCAN;
                }
            }
        }
    /*路口数量检测*/
    //跳到此处
    part1:
        ROS_DEBUG("Jump to Part 1");

        //laser可视数量
        ROS_DEBUG_STREAM("Num of Edge Laser =   " << numOfEdgeLaser);

        if (cloudFar->points.size() >= 20)
            ROS_DEBUG_STREAM("                    Size of Far >= " << cloudFar->points.size());
        if (cloudFar->points.size() >= 40)
        {
            intersectionVerified.data = true;
            ROS_INFO("------------------------------------------------------");
            ROS_INFO("******************Confirm Arriving The Intersections !!!");
        }

        /*排除检测为岔道线的最大光束距离*/
        if (startEnd_Vec.size() == 0)
            return;
        for (std::vector<std::pair<int, int>>::iterator itr = startEnd_Vec.begin(); itr != startEnd_Vec.end(); ++itr)
        {
            double thisMin = 20.0;
            std::vector<std::pair<int, int>>::iterator itr_front;

            if (itr == startEnd_Vec.begin())
            {
                itr_front = startEnd_Vec.end() - 1;
            }
            else
            {
                itr_front = itr - 1;
            }

            if (itr->first < itr_front->second)
            {
                for (int i = 0; i < sizeOfBeamInvalid; ++i)
                {
                    if (i == itr->first)
                        i = itr_front->second;
                    double thisDistance = Distance_plot[i];
                    if (thisMin > 20)
                    {
                        ROS_WARN_STREAM("Divide WARNING =  " << thisMin);
                        break;
                    }
                    if (thisMin > thisDistance)
                        thisMin = thisDistance;
                }
            }
            else
            {
                for (int i = itr_front->second; i < itr->first; ++i)
                {
                    double thisDistance = Distance_plot[i];
                    if (thisMin > 20)
                    {
                        ROS_WARN_STREAM("Divide WARNING =  " << thisMin);
                        break;
                    }
                    if (thisMin > thisDistance)
                        thisMin = thisDistance;
                }
            }
            peakDistance_Vec.push_back(thisMin);
        }
        peakDistance_Max.data = *(std::max_element(peakDistance_Vec.begin(), peakDistance_Vec.end()));

        // if (peak_Num.data > 2)
        // {
        //     intersectionDetected.data = true;
        //     ROS_DEBUG_STREAM("Intersection Number = " << peak_Num.data);
        //     ROS_INFO("------------------------------------------------------");
        //     ROS_INFO("****Detect The Intersections !!!****************");
        // }

        ROS_DEBUG("Intersection Detection Success !!!");
    }

    void intersectionDivide()
    {

        if (outlier_Bool && false)
        {
            // 创建滤波器对象
            pcl::PassThrough<pcl::PointXYZI> pass;
            pass.setInputCloud(cloudWithInfo);
            pass.setFilterFieldName("intensity");
            pass.setFilterLimits(0, INT_MAX);
            pass.setFilterLimitsNegative(false);
            pass.filter(*cloudWithInfo);

            kdtree.setInputCloud(cloudWithInfo);
            extract.setInputCloud(cloudWithInfo);
        }
        else
        {
            kdtree.setInputCloud(laserOrigin_Cloud);
            extract.setInputCloud(laserOrigin_Cloud);
        }

        std::vector<int> pointIdxRadiusSearch;         //保存每个近邻点的索引
        std::vector<float> pointRadiusSquaredDistance; //保存每个近邻点与查找点之间的欧式距离平方

        if (kdtree.radiusSearch(egoPoint, segmentationRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0)
        {
            ROS_ERROR("There is no point nearby !!!");
            return;
        }

        //**分割点云**//
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCenter(new pcl::PointCloud<pcl::PointXYZI>());
        boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(pointIdxRadiusSearch);
        extract.setIndices(index_ptr);
        extract.setNegative(false); //如果设为true,可以提取指定index之外的点云
        extract.filter(*cloudCenter);
        ROS_DEBUG_STREAM("Extract Center Success !!! Center Number = " << cloudCenter->size());
        extract.setNegative(true); //提取指定index之外的点云
        extract.filter(*cloudIntersections);
        ROS_DEBUG_STREAM("Extract Intersection Success !!! Intersection Number = " << cloudIntersections->size());

        ROS_DEBUG("Intersection Divide Success !!!");
    }

    void intersectionCluster()
    {
        if (cloudIntersections->size() == 0)
            return;

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloudIntersections); //创建点云索引向量，用于存储实际的点云信息

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(clusterRadius); //设置近邻搜索的搜索半径
        ec.setMinClusterSize(clusterSize_min); //设置一个聚类需要的最少点数目
        ec.setSearchMethod(tree);              //设置点云的搜索机制
        ec.setInputCloud(cloudIntersections);
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
                    cloudCluster_1->points.push_back(cloudIntersections->points[*pit]);
                cloudCluster_1->width = cloudCluster_1->points.size();
                cloudCluster_1->height = 1;
                cloudCluster_1->is_dense = true;

                break;

            case 2:
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloudCluster_2->points.push_back(cloudIntersections->points[*pit]);
                cloudCluster_2->width = cloudCluster_2->points.size();
                cloudCluster_2->height = 1;
                cloudCluster_2->is_dense = true;
                break;

            case 3:
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloudCluster_3->points.push_back(cloudIntersections->points[*pit]);
                cloudCluster_3->width = cloudCluster_3->points.size();
                cloudCluster_3->height = 1;
                cloudCluster_3->is_dense = true;

                break;

            case 4:
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloudCluster_4->points.push_back(cloudIntersections->points[*pit]);
                cloudCluster_4->width = cloudCluster_4->points.size();
                cloudCluster_4->height = 1;
                cloudCluster_4->is_dense = true;

                break;

            case 5:
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloudCluster_5->points.push_back(cloudIntersections->points[*pit]);
                cloudCluster_5->width = cloudCluster_5->points.size();
                cloudCluster_5->height = 1;
                cloudCluster_5->is_dense = true;

                break;

            default:
                break;
            }
        }
        ROS_DEBUG("Intersection Cluster Success !!!");
    }

    void publishResult()
    {
        if (pubCloudOutlierRemove.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*outlierRemove_Cloud, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "base_link";
            pubCloudOutlierRemove.publish(cloudMsgTemp);
        }

        if (pubCloudWithInfo.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudWithInfo, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "base_link";
            pubCloudWithInfo.publish(cloudMsgTemp);
        }

        if (pubCloudFar.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudFar, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "base_link";
            pubCloudFar.publish(cloudMsgTemp);
        }

        if (pubCloudIntersection.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudIntersections, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "base_link";
            pubCloudIntersection.publish(cloudMsgTemp);
        }

        if (pubCluster_1.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_1, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "base_link";
            pubCluster_1.publish(cloudMsgTemp);
        }

        if (pubCluster_2.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_2, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "base_link";
            pubCluster_2.publish(cloudMsgTemp);
        }

        if (pubCluster_3.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_3, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "base_link";
            pubCluster_3.publish(cloudMsgTemp);
        }

        if (pubCluster_4.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_4, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "base_link";
            pubCluster_4.publish(cloudMsgTemp);
        }

        if (pubCluster_5.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_5, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "base_link";
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

        if (true)
        {
            pubLaserLane.publish(laser_Lane);
            pubEdgeLane.publish(Edge_Lane);
            pubEdgeLane.publish(circle_Lane);
            pubPeakNum.publish(peak_Num);
            pubClusterNum.publish(cluster_Num);
            peakDistanceMax.publish(peakDistance_Max);

            fixedTrans.stamp_ = ros::Time().fromSec(timeLaserCloudNew);
            tfBroadcaster.sendTransform(fixedTrans);

            std_msgs::Float32MultiArray beam_Dis;
            for (int i = 0; i < Horizon_SCAN; ++i)
            {
                beam_Dis.data.push_back(beamDistance_Vec[i]);
            }
            pubBeamDistance.publish(beam_Dis);

            std_msgs::UInt8 segmentationRadius_Uint8;
            segmentationRadius_Uint8.data = segmentationRadius;
            pubSegmentationRadius.publish(segmentationRadius_Uint8);
        }
        ROS_DEBUG("Publish Result Success !!!");
    }

    void run()
    {
        if (receivePoints && over)
        {
            over = false;
            getDynamicParameter();
            if (outlier_Bool)
                cloudWithInfo = getCloudWithInfo(outlierRemove_Cloud, groundScanInd, aboveScanInd, Horizon_SCAN);
            else
                cloudWithInfo = getCloudWithInfo(laserOrigin_Cloud, groundScanInd, aboveScanInd, Horizon_SCAN);
            laser_Visualizaton();
            intersectionDetection();
            if (intersectionVerified.data)
            {
            loop:
                ROS_DEBUG_STREAM("segmentationRadius  ===========    " << segmentationRadius);
                intersectionDivide();
                intersectionCluster();

                if (cluster_Num.data < peak_Num.data && segmentationRadius < segmentationRadius_Max)
                {
                    segmentationRadius += 2;
                    cloudIntersections->clear();
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
        ros::param::get("/intersection/segmentation_radius", segmentationRadius);
        ros::param::get("/intersection/width_threshold", width_Thre);
        ros::param::get("/intersection/distance_threshold", distance_Thre);
        ros::param::get("/intersection/col_minus_threshold", col_minus_Thre);
        ros::param::get("/intersection/cluster_radius", clusterRadius);
        ros::param::get("/intersection/cluster_size_min", clusterSize_min);
        ros::param::get("/intersection/bool_outlier_removal", outlier_Bool);
        ros::param::get("/intersection/bool_median_filter", medianFilter_Bool);
        ros::param::get("/intersection/median_size", median_Size);
        ros::param::get("/intersection/median_coefficient", median_Coefficient);
    }
};

//动态调参
void callback(perception::param_Config &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %d %d %d %d %d %d %f %f %s %s",
             config.segmentation_radius,
             config.width_threshold,
             config.distance_threshold,
             config.col_minus_threshold,
             config.median_size,
             config.cluster_size_min,
             config.cluster_radius,
             config.median_coefficient,
             config.bool_outlier_removal ? "True" : "False",
             config.bool_median_filter ? "True" : "False");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "intersection");

    //动态参数调节
    dynamic_reconfigure::Server<perception::param_Config> server;
    dynamic_reconfigure::Server<perception::param_Config>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("\033[1;32m---->\033[0m Intersection Detection Started.");

    Detection ND;

    ros::Rate rate(4);
    while (ros::ok())
    {
        ros::spinOnce();
        ND.run();
        rate.sleep();
    }

    return 0;
}
