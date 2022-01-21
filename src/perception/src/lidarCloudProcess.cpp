#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>                 // 包含kdtree头文件
#include <pcl/kdtree/kdtree_flann.h>           //kdtree搜索
#include <pcl/filters/extract_indices.h>       //按索引提取
#include <pcl/segmentation/extract_clusters.h> //分割聚类
#include <pcl/filters/passthrough.h>           //直通滤波
#include <pcl/filters/conditional_removal.h>   //条件滤波
#include <pcl/filters/filter.h>                //移除无效点
#include <message_filters/subscriber.h>        //同步接收
#include <message_filters/time_synchronizer.h> //时间同步
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
#include <perception/lidarCloudProcess_Config.h>

// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
//     MySyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

class lidarCloudProcess
{
private:
    ros::NodeHandle nh;

    std::vector<int> mapping;

    std::string lidarTopic_left;
    std::string lidarTopic_right;
    std::string lidarTopic_top;
    std::string vehicle_frame_id;

    // ros::Time time;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOrigin;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCombinedTrans;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudNoCar;
    ros::Time time_st;

    message_filters::Subscriber<sensor_msgs::PointCloud2> subLidarCloudLeft;
    message_filters::Subscriber<sensor_msgs::PointCloud2> subLidarCloudRight;
    message_filters::Subscriber<sensor_msgs::PointCloud2> subLidarCloudTop;

    ros::Subscriber subLidarCloudLeft;
    ros::Subscriber subLidarCloudRight;
    ros::Subscriber subLidarCloudTop;
    ros::Publisher cloudCombined_pub;

    tf::TransformListener listener_top;
    tf::TransformListener listener_left;
    tf::TransformListener listener_right;

    bool lidarTop_bool;

public:
    lidarCloudProcess() : nh("~")
    {
        //加载参数
        nh.param<std::string>("frame_id_", vehicle_frame_id, "vehicle_base_link");
        ROS_INFO("vehicle_frame_id_: %s", vehicle_frame_id.c_str());
        nh.param<std::string>("lidarTopic_left_", lidarTopic_left, "/velodyne_left");
        ROS_INFO("Left lidar topic : %s", lidarTopic_left.c_str());
        nh.param<std::string>("lidarTopic_right_", lidarTopic_right, "/velodyne_right");
        ROS_INFO("Right lidar topic : %s", lidarTopic_right.c_str());
        nh.param<std::string>("lidarTopic_top_", lidarTopic_top, "/velodyne_top");
        ROS_INFO("Top lidar topic : %s", lidarTopic_top.c_str());
        ROS_INFO("----------------------------------------------------------------------");

        auto subLidarCloudLeft = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, lidarTopic_left, 1);
        auto subLidarCloudRight = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, lidarTopic_right, 1);
        auto subLidarCloudTop = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, lidarTopic_top, 1);

        auto sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *subLidarCloudLeft, *subLidarCloudRight, *subLidarCloudTop);

        // message_filters::Subscriber<sensor_msgs::PointCloud2> subLidarCloudLeft(nh, lidarTopic_left, 1);
        // message_filters::Subscriber<sensor_msgs::PointCloud2> subLidarCloudRight(nh, lidarTopic_right, 1);
        // message_filters::Subscriber<sensor_msgs::PointCloud2> subLidarCloudTop(nh, lidarTopic_top, 1);

        // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subLidarCloudLeft, subLidarCloudRight);

        // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subLidarCloudLeft, subLidarCloudRight, subLidarCloudTop);
        sync->registerCallback(boost::bind(&lidarCloudProcess::combineCallback, this, _1, _2,_3));

        // subLidarCloudLeft = nh.subscribe<sensor_msgs::PointCloud2>(lidarTopic_left, 1, &lidarCloudProcess::leftHandler, this);
        // subLidarCloudRight = nh.subscribe<sensor_msgs::PointCloud2>(lidarTopic_right, 1, &lidarCloudProcess::rightHandler, this);
        // subLidarCloudTop = nh.subscribe<sensor_msgs::PointCloud2>(lidarTopic_top, 2, &lidarCloudProcess::topHandler, this);

        cloudCombined_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_Combined", 1);

        cloudNoCar.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudOrigin.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudCombinedTrans.reset(new pcl::PointCloud<pcl::PointXYZI>());
        lidarTop_bool = false;

        cloudOrigin->header.frame_id = vehicle_frame_id;
        cloudCombinedTrans->header.frame_id = vehicle_frame_id;
        cloudNoCar->header.frame_id = vehicle_frame_id;
    }

    // void combineCallback(const sensor_msgs::PointCloud2ConstPtr &msg_left, const sensor_msgs::PointCloud2ConstPtr &msg_right)
    void combineCallback(const sensor_msgs::PointCloud2ConstPtr &msg_left, const sensor_msgs::PointCloud2ConstPtr &msg_right, const sensor_msgs::PointCloud2ConstPtr &msg_top)
    {
        time_st = msg_right->header.stamp;
        pcl::PointCloud<pcl::PointXYZI> lidarCloudThis, cloudTrans;
        cloudTrans.header.frame_id = vehicle_frame_id;
        pcl::fromROSMsg(*msg_left, lidarCloudThis);
        pcl_ros::transformPointCloud(vehicle_frame_id, lidarCloudThis, cloudTrans, listener_left);
        *cloudOrigin += cloudTrans;
        lidarCloudThis.clear();
        cloudTrans.clear();

        pcl::fromROSMsg(*msg_right, lidarCloudThis);
        pcl_ros::transformPointCloud(vehicle_frame_id, lidarCloudThis, cloudTrans, listener_right);
        *cloudOrigin += cloudTrans;
        lidarCloudThis.clear();
        cloudTrans.clear();

        pcl::fromROSMsg(*msg_top, lidarCloudThis);
        pcl_ros::transformPointCloud(vehicle_frame_id, lidarCloudThis, cloudTrans, listener_top);
        *cloudOrigin += cloudTrans;

        cloudOrigin->header.stamp = lidarCloudThis.header.stamp;
    }

    void topHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        time_st = msg->header.stamp;
        pcl::PointCloud<pcl::PointXYZI> lidarCloudThis, cloudTrans;
        pcl::fromROSMsg(*msg, lidarCloudThis);

        pcl_ros::transformPointCloud(vehicle_frame_id, lidarCloudThis, cloudTrans, listener_top);
        cloudTrans.header.frame_id = vehicle_frame_id;

        *cloudOrigin += cloudTrans;
        cloudOrigin->header.stamp = lidarCloudThis.header.stamp;
        if (!cloudTrans.empty())
        {
            lidarTop_bool = true;
        }
    }

    void leftHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        time_st = msg->header.stamp;
        pcl::PointCloud<pcl::PointXYZI> lidarCloudThis, cloudTrans;
        pcl::fromROSMsg(*msg, lidarCloudThis);

        pcl_ros::transformPointCloud(vehicle_frame_id, lidarCloudThis, cloudTrans, listener_left);
        cloudTrans.header.frame_id = vehicle_frame_id;

        *cloudOrigin += cloudTrans;
        cloudOrigin->header.stamp = lidarCloudThis.header.stamp;
    }

    void rightHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        time_st = msg->header.stamp;
        pcl::PointCloud<pcl::PointXYZI> lidarCloudThis, cloudTrans;
        pcl::fromROSMsg(*msg, lidarCloudThis);

        pcl_ros::transformPointCloud(vehicle_frame_id, lidarCloudThis, cloudTrans, listener_right);
        cloudTrans.header.frame_id = vehicle_frame_id;

        *cloudOrigin += cloudTrans;
        cloudOrigin->header.stamp = lidarCloudThis.header.stamp;
    }

    void pointCloudProcess()
    {
        cloudCombinedTrans = cloudOrigin;

        /*动态参数*/
        int segmentationRadius;
        int node_Id;
        bool save_Bool;
        std::string save_Name;

        ros::param::get("/lidarCloudProcess/segmentation_radius", segmentationRadius);
        ros::param::get("/lidarCloudProcess/node_id", node_Id);
        ros::param::get("/lidarCloudProcess/bool_save", save_Bool);
        ros::param::get("/lidarCloudProcess/save_name", save_Name);
        /*动态参数*/

        pcl::PointCloud<pcl::PointXYZI> cloudFinal;

        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        std::vector<int> index;      //保存每个近邻点的索引
        std::vector<float> distance; //保存每个近邻点与查找点之间的欧式距离平方

        pcl::PointXYZI zeroPoint(0.f);
        zeroPoint.x = zeroPoint.y = zeroPoint.z = 0;

        //分离地面点
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCombinedFiltered(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PassThrough<pcl::PointXYZI> groundFilter;
        groundFilter.setInputCloud(cloudCombinedTrans);
        groundFilter.setFilterFieldName("z");
        groundFilter.setFilterLimits(-0.8, 2);
        groundFilter.setFilterLimitsNegative(false);
        groundFilter.filter(*cloudCombinedFiltered);
        // 分割车辆
        pcl::ConditionOr<pcl::PointXYZI>::Ptr conditionOr(new pcl::ConditionOr<pcl::PointXYZI>()); //条件滤波
        pcl::ConditionalRemoval<pcl::PointXYZI> condition;

        conditionOr->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GE, 2.5))); // GT表示大于等于
        conditionOr->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LE, -3)));  // GT表示大于等于
        conditionOr->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GE, 1)));   // LT表示小于等于
        conditionOr->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LE, -1)));  // LT表示小于等于

        pcl::PointCloud<pcl::PointXYZI> cloudTemp;
        condition.setCondition(conditionOr);
        condition.setInputCloud(cloudCombinedFiltered);
        condition.setKeepOrganized(false);
        condition.filter(cloudTemp);

        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(cloudTemp, *cloudNoCar, mapping);

        // 外围分割
        kdtree.setInputCloud(cloudNoCar); // 设置要搜索的点云，建立KDTree
        pcl::ExtractIndices<pcl::PointXYZI> extract_2;
        if (kdtree.radiusSearch(zeroPoint, segmentationRadius, index, distance) == 0)
        {
            ROS_ERROR("There is no point nearby !!!");
            return;
        }
        extract_2.setInputCloud(cloudNoCar);
        boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index);
        extract_2.setIndices(index_ptr);
        extract_2.setNegative(false); //如果设为true,可以提取指定index之外的点云
        extract_2.filter(cloudFinal);

        // 保存文件
        if (save_Bool)
        {
            std::string fileName;
            fileName = "/home/lsj/dev/Mine_WS/src/perception/simu_data/" + std::to_string(node_Id) + save_Name;
            pcl::io::savePCDFileASCII(fileName, cloudFinal); //将点云保存到PCD文件中
            ROS_INFO("PCD file saved in  :  [%s]", fileName.c_str());
        }
    }

    // void vehicleReference_pub()
    // {
    //     // 发布变换
    //     tf::TransformBroadcaster br;
    //     tf::Transform transform;
    //     transform.setOrigin(tf::Vector3(-2, 0, 1));
    //     transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    //     while (ros::ok())
    //     {
    //         tf::StampedTransform st(transform, ros::Time::now(), vehicle_frame_id, "vehicle_reference");
    //         usleep(10000);
    //         br.sendTransform(st);
    //     }
    // }

    // void tfReceive()
    // {
    //     tf::TransformListener listener(ros::Duration(1));
    //     while (ros::ok())
    //     {
    //         // 坐标系转换
    //         if (listener.waitForTransform(vehicle_frame_id, "vehicle_reference", ros::Time(0), ros::Duration(1)))
    //             ROS_INFO("YES Receive Tranform !!!!");
    //         else
    //             ROS_INFO("NO Receive Tranform !!!!");
    //     }
    // }
    void listen2tf()
    {
        listener_left.waitForTransform(vehicle_frame_id, "velodyne_left_base_link", ros::Time(0), ros::Duration(100));
        listener_right.waitForTransform(vehicle_frame_id, "velodyne_right_base_link", ros::Time(0), ros::Duration(100));
        listener_top.waitForTransform(vehicle_frame_id, "velodyne_top_base_link", ros::Time(0), ros::Duration(100));
    }

    void run()
    {

        // if (lidarTop_bool)
        if (!cloudOrigin->empty())
        {
            pointCloudProcess();

            //发布点云
            // cloudNoCar->header.stamp = time_st.toSec();
            cloudNoCar->header.stamp = cloudOrigin->header.stamp;
            sensor_msgs::PointCloud2 output;
            toROSMsg(*cloudNoCar, output);
            cloudCombined_pub.publish(output);

            lidarTop_bool = false;
        }

        cloudOrigin->clear();
        cloudCombinedTrans->clear();
        cloudNoCar->clear();
    }
};

//动态调参
void callback(perception::lidarCloudProcess_Config &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %d %d %s %s",
             config.segmentation_radius,
             config.node_id,
             config.bool_save ? "True" : "False",
             config.save_name.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidarCloudProcess");

    /*动态参数调节*/
    dynamic_reconfigure::Server<perception::lidarCloudProcess_Config> server;
    dynamic_reconfigure::Server<perception::lidarCloudProcess_Config>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
    /*动态参数调节*/

    ROS_INFO("\033[1;32m---->\033[0m Simulation Segmentation Save Started.");

    lidarCloudProcess lidarCloudProcess_obj;

    // std::thread thread_1(vehicleReference_pub);
    // std::thread thread_2(tfReceive);

    lidarCloudProcess_obj.listen2tf();

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        lidarCloudProcess_obj.run();

        rate.sleep();
    }

    // thread_1.join();
    // thread_2.join();

    return 0;
}