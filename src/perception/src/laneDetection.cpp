
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>                  //体素滤波
#include <pcl/filters/passthrough.h>                 //直通滤波
#include <pcl/ModelCoefficients.h>                   //投影滤波
#include <pcl/filters/project_inliers.h>             //投影
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波

#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include <cmath>
#include <stdio.h>
#include <thread>
#include <std_msgs/Bool.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include <dynamic_reconfigure/server.h>
#include <perception/param_Config.h>

class laneDetection
{
private:
    int times = 20;
    int validDistance = 20;
    int x_max = 400;
    int y_max = 800;
    int rank = 2;
    float test_value = 2;
    float distance_Right = -1;
    double timeLaserCloudNew;

    bool receivePoints = false;
    bool over = true;

    std_msgs::Float32MultiArray B_array;
    std_msgs::Float32MultiArray Range;
    std_msgs::Float32 Distance;

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

    pcl::console::TicToc time;
    pcl::PassThrough<pcl::PointXYZI> pass_z;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter_1;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter_2;
    pcl::ProjectInliers<pcl::PointXYZI> projection;
    pcl::ModelCoefficients::Ptr coefficients_1;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> outrem; //统计滤波

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZI> seg; //创建分割对象
    pcl::PointIndices::Ptr inliers;
    pcl::ModelCoefficients::Ptr coefficients_2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane;

    // pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdLeft;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdRight;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdFirst;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    cv::Mat polyfit(std::vector<cv::Point> &in_point, int n);

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

public:
    laneDetection() : nh()
    {
        subLaserCloudNew = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_ma", 1, &laneDetection::laserCloudNewHandler, this);

        pubCloudTFDS = nh.advertise<sensor_msgs::PointCloud2>("TFDS", 1);
        pubCloudHigh = nh.advertise<sensor_msgs::PointCloud2>("cloudHigh", 1);
        pubCloudFinal = nh.advertise<sensor_msgs::PointCloud2>("cloudFinal", 1);
        pubCluster_1 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_1", 1);
        pubCluster_2 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_2", 1);
        pubCluster_3 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_3", 1);
        pubCluster_4 = nh.advertise<sensor_msgs::PointCloud2>("cloudCluster_4", 1);
        pubLaneLeft = nh.advertise<sensor_msgs::PointCloud2>("laneLeft", 1);
        pubLaneRight = nh.advertise<sensor_msgs::PointCloud2>("laneRight", 1);

        pubDistance = nh.advertise<std_msgs::Float32>("Distance", 1);
        pubLaneCoefficient = nh.advertise<std_msgs::Float32MultiArray>("laneCoefficient", 1);
        pubLaneRange = nh.advertise<std_msgs::Float32MultiArray>("laneRange", 1);

        downSizeFilter_1.setLeafSize(0.5, 0.5, 0.5);
        downSizeFilter_2.setLeafSize(1.0, 1.0, 1.0);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-1.9, 0);
        pass_z.setFilterLimitsNegative(false);
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

        pass_z.setInputCloud(laserCloudNew);
        pass_z.filter(*laserCloudNewZ);

        downSizeFilter_1.setInputCloud(laserCloudNewZ);
        downSizeFilter_1.filter(*laserCloudNewDS_1);

        projection.setInputCloud(laserCloudNewDS_1);
        projection.setModelCoefficients(coefficients_1);
        projection.filter(*laserCloudNewTFDS);

        downSizeFilter_1.setInputCloud(laserCloudNewTFDS);
        downSizeFilter_1.filter(*laserCloudNewDS_2);

        outrem.setInputCloud(laserCloudNewDS_2);
        outrem.setMeanK(5);
        outrem.setStddevMulThresh(1);
        outrem.filter(*cloudFinal);

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
        ec.setClusterTolerance(1.0); //设置近邻搜索的搜索半径
        ec.setMinClusterSize(50);    //设置一个聚类需要的最少点数目
        ec.setMaxClusterSize(500);   //设置一个聚类需要的最大点数目
        ec.setSearchMethod(tree);    //设置点云的搜索机制
        ec.setInputCloud(cloudFinal);
        ec.extract(cluster_indices); //从点云中提取聚类，并将点云索引保存在cluster_indices中

        /*为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中。*/
        //迭代访问点云索引cluster_indices，直到分割出所有聚类
        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
            // //创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中

            // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
            ++j;

            switch (j)
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
            if (cloudCluster_1->points[pointSearchInd[0]].y > 0)
            {
                downSizeFilter_2.setInputCloud(cloudCluster_1);
                downSizeFilter_2.filter(*laneLeft);
                downSizeFilter_2.setInputCloud(cloudCluster_2);
                downSizeFilter_2.filter(*laneRight);
            }
            else
            {
                downSizeFilter_2.setInputCloud(cloudCluster_2);
                downSizeFilter_2.filter(*laneLeft);
                downSizeFilter_2.setInputCloud(cloudCluster_1);
                downSizeFilter_2.filter(*laneRight);
            }
        }
        std::cout << "Cluster Result Number =  " << j << " data points." << std::endl;
    }

    void laneDetect()
    {
        //创建用于绘制的深蓝色背景图像
        cv::Mat Lane = cv::Mat::zeros(x_max, y_max, CV_8UC3);
        Lane.setTo(cv::Scalar(0, 0, 100));

        for (int i = 2; i <= 2; ++i)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cur(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointXYZI min; //用于存放三个轴的最小值
            pcl::PointXYZI max; //用于存放三个轴的最大值
            pcl::getMinMax3D(*cloud_cur, min, max);
            switch (i)
            {
            case 1:
                if (cloudCluster_1->empty())
                    continue;
                *cloud_cur = *laneLeft;
                break;

            case 2:
                if (cloudCluster_2->empty())
                    continue;
                *cloud_cur = *laneRight;
                //距右侧墙壁最近点
                kdRight->setInputCloud(cloud_cur);
                kdRight->nearestKSearch(egoPoint, 1, pointSearchInd, pointSearchSqDis);
                distance_Right = sqrt(pow(cloud_cur->points[(pointSearchInd[0])].x, 2) + pow(cloud_cur->points[(pointSearchInd[0])].y, 2));
                // std::cout << "Distance_Right =  " << distance_Right  << std::endl;
                Distance.data = distance_Right;
                Range.data.push_back(max.x);
                Range.data.push_back(min.x);
                ROS_ERROR_STREAM("dis: " << distance_Right << "max: " << max.x << "min: " << min.x);
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

            //输入拟合点
            std::vector<cv::Point> fitPoints;
            for (int j = 0; j < cloud_cur->size(); ++j)
            {
                float xx = cloud_cur->points[j].x;
                float yy = cloud_cur->points[j].y;
                fitPoints.push_back(cv::Point(xx, yy));
            }

            cv::Mat A, B, C, D, N;

            polynomial_curve_fit(fitPoints, rank, N);
            // std::cout << " Lane Matrix = " << N << std::endl;

            switch (i)
            {
            case 1:
                A = N;
                std::cout << " A = " << N << std::endl;
                break;

            case 2:
                B = N;
                std::cout << " B = " << N << std::endl;
                break;

            case 3:
                C = N;
                std::cout << " C = " << N << std::endl;
                break;

            case 4:
                D = N;
                std::cout << " D = " << N << std::endl;
                break;

            default:
                continue;
            }

            for (int j = 0; j < rank + 1; ++j)
            {
                B_array.data.push_back(B.at<double>(j, 0));
            }
            // std::cout << " B_array.data.num = " << B_array.data.size() << std::endl;

            //将拟合点绘制到空白图上
            for (int j = 0; j < fitPoints.size(); ++j)
            {
                cv::Point convert;
                convert.x = times * fitPoints[j].x + 0.5 * y_max;
                convert.y = times * fitPoints[j].y + 0.5 * x_max;
                cv::circle(Lane, convert, 5, cv::Scalar(0, 0, 255), 2, 8, 0);
            }

            //绘制折线
            // cv::polylines(Lane, fitPoints, false, cv::Scalar(0, 255, 255), 1, 8, 0);

            std::vector<cv::Point> points_fitted;

            for (int x = -validDistance; x <= validDistance; ++x)
            {
                double y;
                switch (rank)
                {
                case 1:
                    //一阶
                    y = N.at<double>(0, 0) + N.at<double>(1, 0) * x;
                    break;

                case 2:
                    //二阶
                    y = N.at<double>(0, 0) + N.at<double>(1, 0) * x +
                        N.at<double>(2, 0) * std::pow(x, 2);
                    break;

                case 3:
                    //三阶
                    y = N.at<double>(0, 0) + N.at<double>(1, 0) * x +
                        N.at<double>(2, 0) * std::pow(x, 2) + N.at<double>(3, 0) * std::pow(x, 3);
                    break;

                default:
                    continue;
                }

                points_fitted.push_back(cv::Point(times * x + 0.5 * y_max, times * y + 0.5 * x_max));
            }
            cv::polylines(Lane, points_fitted, false, cv::Scalar(255, 255, 255), 1, 8, 0);

            fitPoints.clear();
            points_fitted.clear();
        }

        cv::Mat dst;
        cv::transpose(Lane, dst);
        cv::flip(dst, Lane, -1);
        cv::imshow("车道线", Lane);

        cv::waitKey(1);
    }

    bool polynomial_curve_fit(std::vector<cv::Point> &key_point, int n, cv::Mat &N)
    {
        //Number of key points
        int Num = key_point.size();

        //构造矩阵X
        cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
        for (int i = 0; i < n + 1; ++i)
        {
            for (int j = 0; j < n + 1; ++j)
            {
                for (int k = 0; k < Num; ++k)
                {
                    X.at<double>(i, j) = X.at<double>(i, j) +
                                         std::pow(key_point[k].x, i + j);
                }
            }
        }

        //构造矩阵Y
        cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
        for (int i = 0; i < n + 1; ++i)
        {
            for (int k = 0; k < Num; ++k)
            {
                Y.at<double>(i, 0) = Y.at<double>(i, 0) +
                                     std::pow(key_point[k].x, i) * key_point[k].y;
            }
        }

        N = cv::Mat::zeros(n + 1, 1, CV_64FC1);
        //求解矩阵A
        cv::solve(X, Y, N, cv::DECOMP_LU);
        return true;
    }

    void publishResult()
    {
        if (pubCloudHigh.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*laserCloudNewDS_1, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCloudHigh.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudWithInfo Published.");
        }

        if (pubCloudTFDS.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*laserCloudNewTFDS, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCloudTFDS.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m Cloud Published.");
        }

        if (pubCloudFinal.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudFinal, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCloudFinal.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        }

        if (pubCluster_1.getNumSubscribers() != 0 && !cloudCluster_1->empty())
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_1, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCluster_1.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        }

        if (pubCluster_2.getNumSubscribers() != 0 && !cloudCluster_2->empty())
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_2, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCluster_2.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        }

        if (pubCluster_3.getNumSubscribers() != 0 && !cloudCluster_3->empty())
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_3, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCluster_3.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        }

        if (pubCluster_4.getNumSubscribers() != 0 && !cloudCluster_4->empty())
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudCluster_4, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubCluster_4.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m cloudFinal Published.");
        }

        if (pubLaneLeft.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*laneLeft, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
            pubLaneLeft.publish(cloudMsgTemp);
            // ROS_INFO("\033[1;32m--->\033[0m laneLeft Published.");
        }

        if (pubLaneRight.getNumSubscribers() != 0 && !laneRight->empty())
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*laneRight, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserCloudNew);
            cloudMsgTemp.header.frame_id = "velodyne";
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

            std::cout << "---------------------------------------------" << std::endl;
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
