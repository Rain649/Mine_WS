
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/time.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h> //体素滤波
// #include <pcl/filters/uniform_sampling.h> //均匀采样
// #include <pcl/filters/grid_minimum.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>                 //直通滤波
#include <pcl/ModelCoefficients.h>                   //投影滤波
#include <pcl/filters/project_inliers.h>             //投影
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波

#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include <cmath>
#include <stdio.h>
#include <thread>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <dynamic_reconfigure/server.h>
#include <perception/param_Config.h>

class laneDetection
{
private:
    float distance_Right = -1;

    int order;
    int minClusterSize;
    double clusterRadius;
    double timeLaserCloudNew;
    double passX_min;
    double passZ_min;
    double passZ_max;
    bool receivePoints = false;
    bool over = true;

    std_msgs::Float32MultiArray B_array;
    std_msgs::Float32MultiArray Range;
    std_msgs::Float32 Distance;

    std::string pointCloud_topic_;
    std::string frame_id_;

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloudNew;

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

    pcl::PassThrough<pcl::PointXYZI> pass_z;
    pcl::PassThrough<pcl::PointXYZI> pass_x;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter_1;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter_2;
    // pcl::UniformSampling<pcl::PointXYZI> uniformFilter;
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximateFilter;
    pcl::ProjectInliers<pcl::PointXYZI> projection;
    pcl::ModelCoefficients::Ptr coefficients_1;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> outrem; //统计滤波

    // pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdLeft;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdRight;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdFirst;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    ros::Publisher pubCloudTFDS;
    ros::Publisher pubCloudHigh;
    ros::Publisher pubCloudFinal;
    ros::Publisher pubCluster_1;
    ros::Publisher pubCluster_2;
    ros::Publisher pubCluster_3;
    ros::Publisher pubCluster_4;
    ros::Publisher pubLaneLeft;
    ros::Publisher pubLaneRight;
    ros::Publisher pubLaneRange;
    ros::Publisher pubLaneCoefficient;
    ros::Publisher pubDistance;
    ros::Publisher pubLaneMarker;

public:
    laneDetection() : nh("~")
    {
        nh.param<std::string>("pointCloud_topic_", pointCloud_topic_, "/velodyne_points_ma");
        ROS_INFO("Input Point Cloud: %s", pointCloud_topic_.c_str());
        nh.param<std::string>("frame_id_", frame_id_, "base_link");
        ROS_INFO("Point Cloud Frame ID: %s", frame_id_.c_str());
        nh.param("order", order, 3);
        ROS_INFO("Poly Fit Order: %d", order);
        nh.param("minClusterSize", minClusterSize, 200);
        ROS_INFO("Minimum ClusterSize: %d", minClusterSize);
        nh.param("clusterRadius", clusterRadius, 0.5);
        ROS_INFO("Cluster Radius: %f", clusterRadius);
        nh.param("passX_min", passX_min, -3.0);
        ROS_INFO("PassThrough Filter X Minimum: %f", passX_min);
        nh.param("passZ_min", passZ_min, -1.8);
        ROS_INFO("PassThrough Filter Z Minimum: %f", passZ_min);
        nh.param("passZ_max", passZ_max, 0.0);
        ROS_INFO("PassThrough Filter Z Maximum: %f", passZ_max);

        subLaserCloudNew = nh.subscribe<sensor_msgs::PointCloud2>(pointCloud_topic_, 1, &laneDetection::laserCloudNewHandler, this);

        pubCloudTFDS = nh.advertise<sensor_msgs::PointCloud2>("TFDS", 1);
        pubCloudHigh = nh.advertise<sensor_msgs::PointCloud2>("cloudHigh", 1);
        pubCloudFinal = nh.advertise<sensor_msgs::PointCloud2>("cloudFinal", 1);
        pubCluster_1 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_1", 1);
        pubCluster_2 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_2", 1);
        pubCluster_3 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_3", 1);
        pubCluster_4 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_4", 1);
        pubLaneLeft = nh.advertise<sensor_msgs::PointCloud2>("laneLeft", 1);
        pubLaneRight = nh.advertise<sensor_msgs::PointCloud2>("laneRight", 1);

        pubLaneMarker = nh.advertise<visualization_msgs::Marker>("laneMarker", 1);

        pubDistance = nh.advertise<std_msgs::Float32>("Distance", 1);
        pubLaneCoefficient = nh.advertise<std_msgs::Float32MultiArray>("laneCoefficient", 1);
        pubLaneRange = nh.advertise<std_msgs::Float32MultiArray>("laneRange", 1);

        downSizeFilter_1.setLeafSize(0.2, 0.2, 0.4);
        downSizeFilter_2.setLeafSize(3.0, 3.0, 3.0);
        // uniformFilter.setRadiusSearch(2.0f);
        approximateFilter.setLeafSize(2.0, 2.0, 2.0);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(passZ_min, passZ_max);
        pass_z.setFilterLimitsNegative(false);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(passX_min, FLT_MAX);
        pass_x.setFilterLimitsNegative(false);

        projection.setModelType(pcl::SACMODEL_PLANE);

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
        // kdLeft.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
        kdRight.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());

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

        Distance.data = -1;
        B_array.data.clear();
        Range.data.clear();

        receivePoints = false;
        over = true;
    }

    void laserCloudNewHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        timeLaserCloudNew = msg->header.stamp.toSec();
        laneRight->clear();
        laserCloudNew->clear();
        laserCloudNewZ->clear();
        laserCloudNewDS_1->clear();
        laserCloudNewDS_2->clear();
        laserCloudNewTFDS->clear();
        pcl::fromROSMsg(*msg, *laserCloudNew);

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

        cluster();

        receivePoints = true;
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
        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            switch (++j)
            {
            case 1:
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloudCluster_1->points.push_back(cloudFinal->points[*pit]);
                cloudCluster_1->width = cloudCluster_1->points.size();
                cloudCluster_1->height = 1;
                cloudCluster_1->is_dense = true;

                break;

            case 2:
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloudCluster_2->points.push_back(cloudFinal->points[*pit]);
                cloudCluster_2->width = cloudCluster_2->points.size();
                cloudCluster_2->height = 1;
                cloudCluster_2->is_dense = true;

                break;

            case 3:
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloudCluster_3->points.push_back(cloudFinal->points[*pit]);
                cloudCluster_3->width = cloudCluster_3->points.size();
                cloudCluster_3->height = 1;
                cloudCluster_3->is_dense = true;

                break;

            case 4:
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloudCluster_4->points.push_back(cloudFinal->points[*pit]);
                cloudCluster_4->width = cloudCluster_4->points.size();
                cloudCluster_4->height = 1;
                cloudCluster_4->is_dense = true;

                break;
            default:
                break;
            }

            if (cloudCluster_1->empty())
                break;

            kdFirst->setInputCloud(cloudCluster_1);
            kdFirst->nearestKSearch(egoPoint, 1, pointSearchInd, pointSearchSqDis);
            double y1 = cloudCluster_1->points[pointSearchInd[0]].y;
            double y2 = 0;
            if (!cloudCluster_2->empty())
            {
                kdFirst->setInputCloud(cloudCluster_2);
                kdFirst->nearestKSearch(egoPoint, 1, pointSearchInd, pointSearchSqDis);
                y2 = cloudCluster_2->points[pointSearchInd[0]].y;
            }

            if (y1 < y2)
                cloudCluster_1->swap(*cloudCluster_2);

            // downSizeFilter_2.setInputCloud(cloudCluster_1);
            // downSizeFilter_2.filter(*laneLeft);
            // downSizeFilter_2.setInputCloud(cloudCluster_2);
            // downSizeFilter_2.filter(*laneRight);

            approximateFilter.setInputCloud(cloudCluster_1);
            approximateFilter.filter(*laneLeft);
            approximateFilter.setInputCloud(cloudCluster_2);
            approximateFilter.filter(*laneRight);

            // pcl::GridMinimum<pcl::PointXYZI> gridFilter(3.0);
            // gridFilter.setInputCloud(cloudCluster_1);
            // gridFilter.filter(*laneLeft);
            // gridFilter.setInputCloud(cloudCluster_2);
            // gridFilter.filter(*laneRight);

            // uniformFilter.setInputCloud(cloudCluster_1);
            // uniformFilter.filter(*laneLeft);
            // uniformFilter.setInputCloud(cloudCluster_2);
            // uniformFilter.filter(*laneRight);
        }
        // std::cout << "Cluster Result Number =  " << j << " data points." << std::endl;
        // std::cout << "Cluster Number 1 =  " << cloudCluster_1->size() << " data points." << std::endl;
        // std::cout << "Cluster Number 2 =  " << cloudCluster_2->size() << " data points." << std::endl;
    }

    void laneDetect()
    {
        Eigen::VectorXd A, B, C, D, N;
        for (size_t i = 2; i <= 2; ++i)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cur(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointXYZI min; //用于存放三个轴的最小值
            pcl::PointXYZI max; //用于存放三个轴的最大值
            switch (i)
            {
            case 1:
                if (cloudCluster_1->empty())
                    continue;
                *cloud_cur = *laneLeft;
                break;

            case 2:
                if (laneRight->empty())
                    continue;
                *cloud_cur = *laneRight;
                //距右侧墙壁最近点
                kdRight->setInputCloud(cloud_cur);
                kdRight->nearestKSearch(egoPoint, 1, pointSearchInd, pointSearchSqDis);
                distance_Right = sqrt(pow(cloud_cur->points[(pointSearchInd[0])].x, 2) + pow(cloud_cur->points[(pointSearchInd[0])].y, 2));
                // std::cout << "Distance_Right =  " << distance_Right  << std::endl;
                Distance.data = distance_Right;
                pcl::getMinMax3D(*cloud_cur, min, max);
                Range.data.push_back(max.x);
                Range.data.push_back(min.x);
                // ROS_ERROR_STREAM("dis: " << distance_Right << " max: " << max.x << " min: " << min.x);
                break;

            case 3:
                if (cloudCluster_3->empty())
                    continue;
                *cloud_cur = *cloudCluster_3;
                break;

            case 4:
                if (cloudCluster_4->empty())
                    continue;
                *cloud_cur = *cloudCluster_4;
                break;

            default:
                continue;
            }

            // //输入拟合点
            Eigen::VectorXd xvals(cloud_cur->size());
            Eigen::VectorXd yvals(cloud_cur->size());
            for (size_t j = 0; j < cloud_cur->size(); ++j)
            {
                xvals[j] = cloud_cur->points[j].x;
                yvals[j] = cloud_cur->points[j].y;
            }

            N = polyfit(xvals, yvals, order);

            switch (i)
            {
            case 1:
                A = N;
                // std::cout << " A = " << N << std::endl;
                break;

            case 2:
                B = N;
                // std::cout << " B = " << N << std::endl;
                break;

            case 3:
                C = N;
                // std::cout << " C = " << N << std::endl;
                break;

            case 4:
                D = N;
                // std::cout << " D = " << N << std::endl;
                break;

            default:
                continue;
            }
        }
        if (B.size() == (order + 1))
        {
            for (size_t j = 0; j <= order; ++j)
                B_array.data.push_back(B(j));
            //绘制曲线
            visualize(B);
        }
    }

    void visualize(const Eigen::VectorXd &coeffs)
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
        for (float x = ceil(Range.data.back()); x <= ceil(Range.data.front()); ++x)
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

        pubLaneMarker.publish(line_strip);
    }

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
        if (pubCloudHigh.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*laserCloudNewDS_1, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = frame_id_;
            pubCloudHigh.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudWithInfo Published.");
        }

        if (pubCloudTFDS.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*laserCloudNewTFDS, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = frame_id_;
            pubCloudTFDS.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m Cloud Published.");
        }

        if (pubCloudFinal.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudFinal, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = frame_id_;
            pubCloudFinal.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        }

        if (pubCluster_1.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_1, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = frame_id_;
            pubCluster_1.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        }

        if (pubCluster_2.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_2, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = frame_id_;
            pubCluster_2.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        }

        if (pubCluster_3.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_3, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = frame_id_;
            pubCluster_3.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        }

        if (pubCluster_4.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_4, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = frame_id_;
            pubCluster_4.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        }

        if (pubLaneLeft.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*laneLeft, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = frame_id_;
            pubLaneLeft.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m laneLeft Published.");
        }

        if (pubLaneRight.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*laneRight, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = frame_id_;
            pubLaneRight.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        }

        if (Distance.data)
        {
            pubDistance.publish(Distance);
            // ROS_INFO("\033[1;32m--->\033[0m Distance Published.");
        }

        if (!B_array.data.empty())
        {
            pubLaneCoefficient.publish(B_array);
            // ROS_INFO("\033[1;32m--->\033[0m Coefficient Published.");
        }
        if (!Range.data.empty())
        {
            pubLaneRange.publish(Range);
            // ROS_INFO("\033[1;32m--->\033[0m Coefficient Published.");
        }
    }

    void run()
    {
        if (receivePoints && over)
        {
            over = false;
            laneDetect();
            publishResult();
            clearMemory();

            // std::cout << "---------------------------------------------" << std::endl;
        }
    }
};

void callback(perception::param_Config &config, uint32_t level)
{
    //   ROS_INFO("Reconfigure Request: %d %f %s %s %d",
    //             config.int_param,
    //             config.double_param,
    //             config.str_param.c_str(),
    //             config.bool_param? "True" : "False",
    //             config.size);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "laneDetection");

    //动态参数调节
    dynamic_reconfigure::Server<perception::param_Config> server;
    dynamic_reconfigure::Server<perception::param_Config>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("\033[1;32m---->\033[0m Lane Detection Started.");

    laneDetection LD;

    ros::Rate rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        LD.run();
        rate.sleep();
    }

    return 0;
}
