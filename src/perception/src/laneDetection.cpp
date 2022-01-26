
#include <ros/ros.h>
// #include <omp.h>
#include <sensor_msgs/PointCloud2.h>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/time.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>                  //体素滤波
#include <pcl/filters/passthrough.h>                 //直通滤波
#include <pcl/ModelCoefficients.h>                   //投影滤波
#include <pcl/filters/project_inliers.h>             //投影
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波

#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <vector>

#include <cmath>
#include <stdio.h>
#include <thread>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <dynamic_reconfigure/server.h>
#include <perception/laneDetection_Config.h>

class laneDetection
{
private:
    int order;
    int minClusterSize;

    double ground_remove_height;
    double clusterRadius;
    double timeLaserCloudNew;
    double passX_min;
    double passZ_max;

    bool intersectionVerified;
    // bool leftLane_bool;
    // bool rightLane_bool;

    /// *poly fit left coefficients
    std_msgs::Float32MultiArray leftCoefficient_array;
    /// *poly fit right coefficients
    std_msgs::Float32MultiArray rightCoefficient_array;
    /// *[front distance, back distance]
    std_msgs::Float32MultiArray leftRange;
    /// *[front distance, back distance]
    std_msgs::Float32MultiArray rightRange;
    /// *[left distance, right distance]
    std_msgs::Float32MultiArray Distance;

    std::string pointCloud_topic_;
    std::string frame_id_;

    ros::NodeHandle nh;

    pcl::PointXYZI egoPoint;

    pcl::PointXY ExistPoint;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNew;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNewZ;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNewDS_1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNewDS_2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNewTFDS;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laneLeft;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laneRight;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudWithInfo;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFinal;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_3;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_4;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloudCluster_vec;
    std::vector<float> cloudDistance_vec;
    std::vector<float> yDis_vec;

    pcl::PassThrough<pcl::PointXYZI> pass_z;
    pcl::PassThrough<pcl::PointXYZI> pass_x;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter_1;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter_2;
    pcl::ProjectInliers<pcl::PointXYZI> projection;
    pcl::ModelCoefficients::Ptr coefficients_1;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> outrem; //统计滤波

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdFirst;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    ros::Subscriber subLaserCloudNew;
    ros::Subscriber subIntersection;

    ros::Publisher pubCloudTFDS;
    ros::Publisher pubCloudHigh;
    ros::Publisher pubCloudFinal;
    ros::Publisher pubCluster_1;
    ros::Publisher pubCluster_2;
    ros::Publisher pubCluster_3;
    ros::Publisher pubCluster_4;
    ros::Publisher pubLaneLeft;
    ros::Publisher pubLaneRight;
    ros::Publisher pubLeftRange;
    ros::Publisher pubRightRange;
    ros::Publisher pubLeftCoefficient;
    ros::Publisher pubRightCoefficient;
    ros::Publisher pubDistance;
    ros::Publisher pubLaneLeftMarker;
    ros::Publisher pubLaneRightMarker;

public:
    laneDetection() : nh("~")
    {
        //加载参数
        nh.param<double>("ground_remove_height_", ground_remove_height, -1.0);
        ROS_INFO("Ground remove height : %f", ground_remove_height);
        nh.param<std::string>("pointCloud_topic_", pointCloud_topic_, "/lidarCloudProcess/cloud_Combined");
        ROS_INFO("Input Point Cloud: %s", pointCloud_topic_.c_str());
        nh.param<std::string>("frame_id_", frame_id_, "vehicle_base_link");
        ROS_INFO("Point Cloud Frame ID: %s", frame_id_.c_str());
        nh.param("order", order, 3);
        ROS_INFO("Poly Fit Order: %d", order);
        nh.param("minClusterSize", minClusterSize, 80); // 100
        ROS_INFO("Minimum ClusterSize: %d", minClusterSize);
        nh.param("clusterRadius", clusterRadius, 0.8); // 1.0
        ROS_INFO("Cluster Radius: %f", clusterRadius);
        nh.param("passX_min", passX_min, -5.0);
        ROS_INFO("PassThrough Filter X Minimum: %f", passX_min);
        nh.param("passZ_max", passZ_max, 1.0);
        ROS_INFO("PassThrough Filter Z Maximum: %f", passZ_max);

        subLaserCloudNew = nh.subscribe<sensor_msgs::PointCloud2>(pointCloud_topic_, 1, &laneDetection::laserCloudNewHandler, this);
        subIntersection = nh.subscribe<std_msgs::Bool>("/intersectionDetection/intersectionVerified", 1, &laneDetection::intersectionHandler, this);

        pubCloudTFDS = nh.advertise<sensor_msgs::PointCloud2>("TFDS", 1);
        pubCloudHigh = nh.advertise<sensor_msgs::PointCloud2>("cloudHigh", 1);
        pubCloudFinal = nh.advertise<sensor_msgs::PointCloud2>("cloudFinal", 1);
        pubCluster_1 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_1", 1);
        pubCluster_2 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_2", 1);
        pubCluster_3 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_3", 1);
        pubCluster_4 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_4", 1);
        pubLaneLeft = nh.advertise<sensor_msgs::PointCloud2>("laneLeft", 1);
        pubLaneRight = nh.advertise<sensor_msgs::PointCloud2>("laneRight", 1);

        pubLaneLeftMarker = nh.advertise<visualization_msgs::Marker>("laneLeftMarker", 1);
        pubLaneRightMarker = nh.advertise<visualization_msgs::Marker>("laneRightMarker", 1);

        pubDistance = nh.advertise<std_msgs::Float32MultiArray>("Distance", 1);
        pubLeftCoefficient = nh.advertise<std_msgs::Float32MultiArray>("leftCoefficient", 1);
        pubRightCoefficient = nh.advertise<std_msgs::Float32MultiArray>("rightCoefficient", 1);
        pubLeftRange = nh.advertise<std_msgs::Float32MultiArray>("leftRange", 1);
        pubRightRange = nh.advertise<std_msgs::Float32MultiArray>("rightRange", 1);

        downSizeFilter_1.setLeafSize(0.2, 0.2, 0.4);
        downSizeFilter_2.setLeafSize(2.0, 2.0, 2.0);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(ground_remove_height, passZ_max);
        pass_z.setFilterLimitsNegative(false);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(passX_min, FLT_MAX);
        pass_x.setFilterLimitsNegative(false);

        projection.setModelType(pcl::SACMODEL_PLANE);

        intersectionVerified = false;
        // leftLane_bool = false;
        // rightLane_bool = false;

        allocateMemory();
    }

    void allocateMemory()
    {
        egoPoint.x = 0;
        egoPoint.y = 0;
        egoPoint.z = 0;

        laserCloudNew.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laserCloudNewZ.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laserCloudNewDS_1.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laserCloudNewDS_2.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laserCloudNewTFDS.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudWithInfo.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laneLeft.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laneRight.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudFinal.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudCluster_1.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudCluster_2.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudCluster_3.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudCluster_4.reset(new pcl::PointCloud<pcl::PointXYZI>());

        kdFirst.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());

        coefficients_1.reset(new pcl::ModelCoefficients());
        coefficients_1->values.resize(4);
        coefficients_1->values[0] = coefficients_1->values[1] = 0;
        coefficients_1->values[2] = 1.0;
        coefficients_1->values[3] = 0;
    }

    void clearMemory()
    {
        laserCloudNew->clear();
        laserCloudNewZ->clear();
        laserCloudNewDS_1->clear();
        laserCloudNewDS_2->clear();
        laserCloudNewTFDS->clear();
        laneLeft->clear();
        laneRight->clear();
        cloudFinal->clear();
        cloudCluster_1->clear();
        cloudCluster_2->clear();
        cloudCluster_3->clear();
        cloudCluster_4->clear();
        cloudCluster_vec.clear();
        cloudDistance_vec.clear();
        yDis_vec.clear();

        Distance.data.clear();
        leftCoefficient_array.data.clear();
        rightCoefficient_array.data.clear();
        leftRange.data.clear();
        rightRange.data.clear();
        // leftLane_bool = false;
        // rightLane_bool = false;
    }

    void laserCloudNewHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        timeLaserCloudNew = msg->header.stamp.toSec();
        // ROS_INFO_STREAM("timeLaserCloudNew : " << timeLaserCloudNew);
        laneRight->clear();
        laserCloudNew->clear();
        laserCloudNewZ->clear();
        laserCloudNewDS_1->clear();
        laserCloudNewDS_2->clear();
        laserCloudNewTFDS->clear();
        pcl::fromROSMsg(*msg, *laserCloudNew);
        if (laserCloudNew->empty())
            return;

        /*Filter with height(z)*/
        pass_z.setInputCloud(laserCloudNew);
        pass_z.filter(*laserCloudNewZ);

        /*Down Size Filter*/
        downSizeFilter_1.setInputCloud(laserCloudNewZ);
        downSizeFilter_1.filter(*laserCloudNewDS_1);

        /*Project to 2D*/
        projection.setInputCloud(laserCloudNewDS_1);
        projection.setModelCoefficients(coefficients_1);
        projection.filter(*laserCloudNewTFDS);

        /*Down Size Filter*/
        downSizeFilter_1.setInputCloud(laserCloudNewTFDS);
        downSizeFilter_1.filter(*laserCloudNewDS_2);

        /*Outlier Filter*/
        outrem.setInputCloud(laserCloudNewDS_2);
        outrem.setMeanK(5);
        outrem.setStddevMulThresh(1);
        outrem.filter(*cloudFinal);

        /*Filter with (x)*/
        pass_x.setInputCloud(cloudFinal);
        pass_x.filter(*cloudFinal);
    }

    void intersectionHandler(const std_msgs::Bool msg)
    {
        intersectionVerified = msg.data;
    }

    void cluster()
    {
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloudFinal); //创建点云索引向量，用于存储实际的点云信息

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(clusterRadius); //设置近邻搜索的搜索半径
        ec.setMinClusterSize(minClusterSize);  //设置一个聚类需要的最少点数目
        ec.setMaxClusterSize(1000);            //设置一个聚类需要的最大点数目
        ec.setSearchMethod(tree);              //设置点云的搜索机制
        ec.setInputCloud(cloudFinal);
        ec.extract(cluster_indices); //从点云中提取聚类，并将点云索引保存在cluster_indices中

        /*为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中。*/
        //迭代访问点云索引cluster_indices，直到分割出所有聚类
        // int j = 0;
        // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        // {
        //     switch (++j)
        //     {
        //     case 1:
        //         for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        //             cloudCluster_1->points.push_back(cloudFinal->points[*pit]);

        //         break;

        //     case 2:
        //         for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        //             cloudCluster_2->points.push_back(cloudFinal->points[*pit]);

        //         break;

        //     case 3:
        //         for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        //             cloudCluster_3->points.push_back(cloudFinal->points[*pit]);

        //         break;

        //     case 4:
        //         for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        //             cloudCluster_4->points.push_back(cloudFinal->points[*pit]);

        //         break;
        //     default:
        //         break;
        //     }
        // }

        // if (cloudCluster_1->empty())
        //     return;

        // kdFirst->setInputCloud(cloudCluster_1);
        // kdFirst->nearestKSearch(egoPoint, 1, pointSearchInd, pointSearchSqDis);

        // float distance_1 = std::sqrt(pointSearchSqDis[0]);
        // double y1 = cloudCluster_1->points[pointSearchInd[0]].y;
        // double y2 = 0;

        // if (!cloudCluster_2->empty())
        // {
        //     kdFirst->setInputCloud(cloudCluster_2);
        //     kdFirst->nearestKSearch(egoPoint, 1, pointSearchInd, pointSearchSqDis);
        //     y2 = cloudCluster_2->points[pointSearchInd[0]].y;
        // }

        // // y2左侧距离，y1右侧距离
        // if (y1 < y2)
        // {
        //     cloudCluster_1->swap(*cloudCluster_2);
        //     Distance.data.push_back(y2);
        //     Distance.data.push_back(y1);
        // }
        // // y1左侧距离，y2右侧距离
        // else
        // {
        //     Distance.data.push_back(y1);
        //     Distance.data.push_back(y2);
        // }

        // downSizeFilter_2.setInputCloud(cloudCluster_1);
        // downSizeFilter_2.filter(*laneLeft);
        // downSizeFilter_2.setInputCloud(cloudCluster_2);
        // downSizeFilter_2.filter(*laneRight);

        // std::cout << "Cluster Result Number =  " << j << " data points." << std::endl;
        // std::cout << "Cluster Number 1 =  " << cloudCluster_1->size() << " data points." << std::endl;
        // std::cout << "Cluster Number 2 =  " << cloudCluster_2->size() << " data points." << std::endl;
        // -----------------------------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_temp(new pcl::PointCloud<pcl::PointXYZI>());
        int numMax = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                cloudCluster_temp->points.push_back(cloudFinal->points[*pit]);
            }
            if (!cloudCluster_temp->empty())
                cloudCluster_vec.push_back(*cloudCluster_temp);
            cloudCluster_temp->clear();
            if (++numMax > 4)
                break;
        }
        size_t clusterNum = cloudCluster_vec.size();
        if (0 == clusterNum)
            return;

        //根据最近点距离排序，取前两个
        // for (auto thisCloud : cloudCluster_vec)
        for (auto itr = cloudCluster_vec.begin(); itr != cloudCluster_vec.end();)
        {
            pcl::PointCloud<pcl::PointXYZI> thisCloud = *itr;
            kdFirst->setInputCloud(thisCloud.makeShared());
            kdFirst->nearestKSearch(egoPoint, 1, pointSearchInd, pointSearchSqDis);
            float nearestDistance = std::sqrt(pointSearchSqDis[0]);
            if (nearestDistance < 10.f)
            {
                cloudDistance_vec.push_back(nearestDistance);
                yDis_vec.push_back(thisCloud.points[pointSearchInd[0]].y);
                ++itr;
            }
            else
            {
                itr = cloudCluster_vec.erase(itr);
                --clusterNum;
            }
        }
        if (0 == clusterNum)
            return;

        // ROS_INFO("yDis_vec.size() %lu", clusterNum);
        //冒泡排序
        if (clusterNum > 1)
            for (size_t i = 0; i < clusterNum - 1; ++i)
            {
                for (size_t j = 0; j < clusterNum - 1 - i; ++j)
                {
                    if (abs(yDis_vec[j]) > abs(yDis_vec[j + 1]))
                    {
                        int tmp = yDis_vec[j];
                        yDis_vec[j] = yDis_vec[j + 1];
                        yDis_vec[j + 1] = tmp;
                        cloudCluster_vec[j].swap(cloudCluster_vec[j + 1]);
                    }
                }
            }

        double y0 = yDis_vec[0];
        double y1 = (clusterNum >= 2) ? yDis_vec[1] : 0;

        // y1左侧距离，y0右侧距离
        if (y0 < y1)
        {
            downSizeFilter_2.setInputCloud(cloudCluster_vec[0].makeShared());
            downSizeFilter_2.filter(*laneRight);
            Distance.data.push_back(y0);
            // rightLane_bool = true;
            if (clusterNum > 1)
            {
                downSizeFilter_2.setInputCloud(cloudCluster_vec[1].makeShared());
                downSizeFilter_2.filter(*laneLeft);
                Distance.data.push_back(y1);
                // leftLane_bool = true;
            }
        }
        // y0左侧距离，y1右侧距离
        else
        {
            downSizeFilter_2.setInputCloud(cloudCluster_vec[0].makeShared());
            downSizeFilter_2.filter(*laneLeft);
            Distance.data.push_back(y0);
            // leftLane_bool = true;
            if (clusterNum > 1)
            {
                downSizeFilter_2.setInputCloud(cloudCluster_vec[1].makeShared());
                downSizeFilter_2.filter(*laneRight);
                Distance.data.push_back(y1);
                // rightLane_bool = true;
            }
        }
    }

    /**
     * 根据聚类车道线点云拟合车道线
     */
    void laneDetect()
    {
        Eigen::VectorXd leftCoefficients, rightCoefficients;

        // #pragma omp parallel sections
        {
            if (!laneLeft->empty())
            // #pragma omp section
            {
                pcl::PointXYZI min; //用于存放三个轴的最小值
                pcl::PointXYZI max; //用于存放三个轴的最大值
                pcl::getMinMax3D(*laneLeft, min, max);
                leftRange.data.push_back(max.x);
                leftRange.data.push_back(min.x);
                // ROS_ERROR_STREAM("dis: " << distance_Right << " max: " << max.x << " min: " << min.x);

                // //输入拟合点
                Eigen::VectorXd xvals(laneLeft->size());
                Eigen::VectorXd yvals(laneLeft->size());
                for (size_t j = 0; j < laneLeft->size(); ++j)
                {
                    xvals[j] = laneLeft->points[j].x;
                    yvals[j] = laneLeft->points[j].y;
                }

                leftCoefficients = polyfit(xvals, yvals, order);
                if (leftCoefficients.size() == (order + 1))
                {
                    for (size_t j = 0; j <= order; ++j)
                        leftCoefficient_array.data.push_back(leftCoefficients(j));
                    //绘制曲线
                    visualization_msgs::Marker line_strip = visualize(leftCoefficients, leftRange);
                    pubLaneLeftMarker.publish(line_strip);
                }
            }
            if (!laneRight->empty())
            // #pragma omp section
            {
                pcl::PointXYZI min; //用于存放三个轴的最小值
                pcl::PointXYZI max; //用于存放三个轴的最大值
                pcl::getMinMax3D(*laneRight, min, max);
                rightRange.data.push_back(max.x);
                rightRange.data.push_back(min.x);
                // ROS_ERROR_STREAM("dis: " << distance_Right << " max: " << max.x << " min: " << min.x);

                // //输入拟合点
                Eigen::VectorXd xvals(laneRight->size());
                Eigen::VectorXd yvals(laneRight->size());
                for (size_t j = 0; j < laneRight->size(); ++j)
                {
                    xvals[j] = laneRight->points[j].x;
                    yvals[j] = laneRight->points[j].y;
                }

                rightCoefficients = polyfit(xvals, yvals, order);
                if (rightCoefficients.size() == (order + 1))
                {
                    for (size_t j = 0; j <= order; ++j)
                        rightCoefficient_array.data.push_back(rightCoefficients(j));
                    //绘制曲线
                    visualization_msgs::Marker line_strip = visualize(rightCoefficients, rightRange);
                    pubLaneRightMarker.publish(line_strip);
                }
            }
        }
    }

    /**
     * 车道线拟合可视化
     * @param coeffs 拟合曲线参数
     * @param range 拟合曲线x轴范围
     */
    visualization_msgs::Marker visualize(const Eigen::VectorXd &coeffs, const std_msgs::Float32MultiArray &range)
    {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = frame_id_;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "lanes";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.2;
        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Create the vertices for the points and lines
        for (float x = ceil(range.data.back()); x <= ceil(range.data.front()); ++x)
        {
            float y = 0;
            for (size_t i = 0; i <= order; ++i)
            {
                y += coeffs(i) * pow(x, i);
            }
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = 0;

            line_strip.points.push_back(p);
        }
        return line_strip;
    }

    /**
     * 车道线最小二乘法拟合
     * @param xvals x坐标向量
     * @param yvals y坐标向量
     * @param order 拟合阶数
     */
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, const int order)
    {
        Eigen::VectorXd result;
        if (order >= xvals.size())
            return result;
        Eigen::MatrixXd A(xvals.size(), order + 1);
        for (int i = 0; i < xvals.size(); ++i)
        {
            A(i, 0) = 1.0;
        }
        for (int j = 0; j < xvals.size(); ++j)
        {
            for (int i = 0; i < order; ++i)
            {
                A(j, i + 1) = A(j, i) * xvals(j);
            }
        }
        auto Q = A.householderQr();
        result = Q.solve(yvals);
        return result;
    }

    void publishResult()
    {
        // if (pubCloudHigh.getNumSubscribers() != 0)
        // {
        //     sensor_msgs::PointCloud2 cloudMsgTemp;
        //     pcl::toROSMsg(*laserCloudNewDS_1, cloudMsgTemp);
        //     // cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
        //     cloudMsgTemp.header.frame_id = frame_id_;
        //     pubCloudHigh.publish(cloudMsgTemp);
        //     // ROS_INFO("\033[1;32m--->\033[0m cloudWithInfo Published.");
        // }

        // if (pubCloudTFDS.getNumSubscribers() != 0)
        // {
        //     sensor_msgs::PointCloud2 cloudMsgTemp;
        //     pcl::toROSMsg(*laserCloudNewTFDS, cloudMsgTemp);
        //     // cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
        //     cloudMsgTemp.header.frame_id = frame_id_;
        //     pubCloudTFDS.publish(cloudMsgTemp);
        //     // ROS_INFO("\033[1;32m--->\033[0m Cloud Published.");
        // }

        // if (pubCloudFinal.getNumSubscribers() != 0)
        // {
        //     sensor_msgs::PointCloud2 cloudMsgTemp;
        //     pcl::toROSMsg(*cloudFinal, cloudMsgTemp);
        //     // cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
        //     cloudMsgTemp.header.frame_id = frame_id_;
        //     pubCloudFinal.publish(cloudMsgTemp);
        //     // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        // }

        if (pubCluster_1.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(cloudCluster_vec[0], cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = frame_id_;
            pubCluster_1.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        }

        if (pubCluster_2.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(cloudCluster_vec[1], cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = frame_id_;
            pubCluster_2.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        }

        if (pubCluster_3.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(cloudCluster_vec[2], cloudMsgTemp);
            // cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = frame_id_;
            pubCluster_3.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        }

        // if (pubCluster_4.getNumSubscribers() != 0)
        // {
        //     sensor_msgs::PointCloud2 cloudMsgTemp;
        //     pcl::toROSMsg(*cloudCluster_4, cloudMsgTemp);
        //     // cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
        //     cloudMsgTemp.header.frame_id = frame_id_;
        //     pubCluster_4.publish(cloudMsgTemp);
        //     // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        // }

        // if (pubLaneLeft.getNumSubscribers() != 0)
        // {
        //     sensor_msgs::PointCloud2 cloudMsgTemp;
        //     pcl::toROSMsg(*laneLeft, cloudMsgTemp);
        //     cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
        //     cloudMsgTemp.header.frame_id = frame_id_;
        //     pubLaneLeft.publish(cloudMsgTemp);
        //     // ROS_INFO("\033[1;32m--->\033[0m laneLeft Published.");
        // }

        // if (pubLaneRight.getNumSubscribers() != 0)
        // {
        //     sensor_msgs::PointCloud2 cloudMsgTemp;
        //     pcl::toROSMsg(*laneRight, cloudMsgTemp);
        //     cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
        //     cloudMsgTemp.header.frame_id = frame_id_;
        //     pubLaneRight.publish(cloudMsgTemp);
        //     // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        // }
        // ---------------------------------------

        if (!Distance.data.empty())
        {
            pubDistance.publish(Distance);
            // ROS_INFO("\033[1;32m--->\033[0m Distance Published.");
        }
        if (!leftCoefficient_array.data.empty())
        {
            pubLeftCoefficient.publish(leftCoefficient_array);
            // ROS_INFO("\033[1;32m--->\033[0m Coefficient Published.");
        }
        if (!rightCoefficient_array.data.empty())
        {
            pubRightCoefficient.publish(rightCoefficient_array);
            // ROS_INFO("\033[1;32m--->\033[0m Coefficient Published.");
        }
        if (!leftRange.data.empty())
        {
            pubLeftRange.publish(leftRange);
            // ROS_INFO("\033[1;32m--->\033[0m Coefficient Published.");
        }
        if (!rightRange.data.empty())
        {
            pubRightRange.publish(rightRange);
            // ROS_INFO("\033[1;32m--->\033[0m Coefficient Published.");
        }
    }

    void run()
    {
        if (intersectionVerified || cloudFinal->empty())
            return;
        // getDynamicParameter();
        cluster();
        laneDetect();
        publishResult();
        clearMemory();
    }

    void getDynamicParameter()
    {
        ros::param::get("/laneDetection/minClusterSize_", minClusterSize);
        ros::param::get("/laneDetection/clusterRadius_", clusterRadius);
    }
};

void callback(perception::laneDetection_Config &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %d %f",
             config.minClusterSize_,
             config.clusterRadius_);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "laneDetection");

    //动态参数调节
    dynamic_reconfigure::Server<perception::laneDetection_Config> server;
    dynamic_reconfigure::Server<perception::laneDetection_Config>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("\033[1;32m---->\033[0m Lane Detection Started.");

    laneDetection LD;

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        LD.run();

        rate.sleep();
    }

    return 0;
}
