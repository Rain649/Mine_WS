#include "dubins_bug_delta/dubins_bug.h"

DBPlanner::DBPlanner(ros::NodeHandle& nh)
{
    // subscribe
    std::string topic_odom;
    nh.getParam("topic_odom", topic_odom);
    OdomSub = nh.subscribe(topic_odom, 10, &DBPlanner::OdomCallback, this);
    
    std::string topic_target;
    nh.getParam("topic_target", topic_target);
    TargetSub = nh.subscribe(topic_target, 10, &DBPlanner::TargetCallback, this);
    
    std::string topic_lidar;
    nh.getParam("topic_lidar", topic_lidar);
    LidarSub = nh.subscribe(topic_lidar, 10, &DBPlanner::LidarCallback, this);
    
    // callback flag
    odom_flag = false;

    target_flag = false;

    lidar_flag = false;

    // path
    PathLocalPub = nh.advertise<nav_msgs::Path>("/path_local", 10);
    nh.getParam("frame_id", path_local_msg.header.frame_id);
    // path_local_msg.header.frame_id = "map";
    
    PathLocalVehPub = nh.advertise<nav_msgs::Path>("/path_local_veh", 10);
    nh.getParam("frame_id", path_local_veh_msg.header.frame_id);
    // path_local_veh_msg.header.frame_id = "map";

    PathPdtPub = nh.advertise<nav_msgs::Path>("/path_pdt", 10);
    nh.getParam("frame_id", path_pdt_msg.header.frame_id);
    // path_pdt_msg.header.frame_id = "map";

    std::string topic_path_pdt_veh;
    nh.getParam("topic_path_pdt", topic_path_pdt_veh);
    PathPdtVehPub = nh.advertise<nav_msgs::Path>(topic_path_pdt_veh, 10);
    nh.getParam("frame_id", path_pdt_veh_msg.header.frame_id);
    // path_pdt_veh_msg.header.frame_id = "map";
    
    // map
    map_rows = 200;
    map_cols = 200;
    map_res = 0.2;
    map_o_x = -20; // 在车辆坐标系下的地图原点坐标 
    map_o_y = -20;

    image_map.create(map_rows, map_cols, CV_8UC1);

    MapPub = nh.advertise<nav_msgs::OccupancyGrid>("/map_local", 1);
    nh.getParam("frame_id", map_msg.header.frame_id);
    // map_msg.header.frame_id = "map";
    
    map_msg.info.resolution = map_res;
    map_msg.info.width = map_cols;
    map_msg.info.height = map_rows;
    map_msg.info.origin.position.x = map_o_x;
    map_msg.info.origin.position.y = map_o_y;
    
    map_vec.resize(map_msg.info.width*map_msg.info.height);

    // lidar
    Laser_Edge = 20;

    pass_z.setFilterFieldName ("z");                        //设置过滤时所需要点云类型的Z字段
    pass_z.setFilterLimits (-10, 1);                        //设置在过滤字段的范围    
    pass_x.setFilterFieldName ("x");                        //设置过滤时所需要点云类型的Z字段
    pass_x.setFilterLimits (-Laser_Edge, Laser_Edge);       //设置在过滤字段的范围
    pass_y.setFilterFieldName ("y");                        //设置过滤时所需要点云类型的Z字段
    pass_y.setFilterLimits (-Laser_Edge, Laser_Edge);       //设置在过滤字段的范围
    downSizeFilter.setLeafSize(map_res, map_res, map_res);  // 分辨率
    
    LaserCloudSurround.reset(new pcl::PointCloud<PointType>());
    LaserCloudSurroundFiltered.reset(new pcl::PointCloud<PointType>());

    ANGLE_RES = M_PI/180;

    seg_num = 2*M_PI/ANGLE_RES;

    Neighbor_num = 4;

    Edge_tolerance = 3;

    neighbor_flag.resize(Neighbor_num);

    nh.getParam("k_safe", k_safe);
    // k_safe = 1.3;//2;

    k_sample = 2;

    laser_point.resize(seg_num);

    LaserPointPub = nh.advertise<geometry_msgs::PoseArray>("/laser_points",2);

    laser_point_msg.poses.resize(laser_point.size());
    
    nh.getParam("frame_id", laser_point_msg.header.frame_id);
    // laser_point_msg.header.frame_id = "map";

    LaserEdgePointPub = nh.advertise<geometry_msgs::PoseArray>("/laser_edge",2);
    
    nh.getParam("frame_id", laser_edge_msg.header.frame_id);
    // laser_edge_msg.header.frame_id = "map";

    // vehicle
    Delta_f_max = 20*M_PI/180;

    nh.getParam("Lf", Lf);
    // Lf = 1.81;

    nh.getParam("Lr", Lr);
    // Lr = 1.69;
    
    Veh_L = Lf + Lr; //3.5;

    // dubins
    steer_radius = Veh_L/tan(Delta_f_max);
    
    step_size = map_res;

    db_rs_flag = false;

    start_state_veh[0] = 0;
    start_state_veh[1] = 0;
    start_state_veh[2] = 0;

    // dis vars
    Infinity = 1.0e4;

    // tolerance
    nh.getParam("Tar_tolerance", Tar_tolerance);
    // Tar_tolerance = 1;
    
    // based on TangentBug
    dis_v2t_old = Infinity;

    // 碰撞检测
    nh.getParam("OBS_DIS", OBS_DIS);
    // OBS_DIS = 1;

    // 更新预测路径
    Phi_alter = M_PI/6;

    S_proportion = 10;

    odom_counter = 0;

    target_counter = 0;

    lidar_counter = 0;

    Counter_MAX = 1 * 20;

    // test
    TargetInVehPub = nh.advertise<nav_msgs::Odometry>("/target_veh", 10);
    nh.getParam("frame_id", target_veh_msg.header.frame_id);
    // target_veh_msg.header.frame_id = "map";
    // test
}

void DBPlanner::OdomCallback(const nav_msgs::Odometry& msg)
{
    current_state.x = msg.pose.pose.position.x;
    current_state.y = msg.pose.pose.position.y;
    current_state.u = msg.twist.twist.linear.x;
    current_state.w = msg.twist.twist.angular.z;
    current_state.yaw = tf::getYaw(msg.pose.pose.orientation);

    start_state[0] = msg.pose.pose.position.x;
    start_state[1] = msg.pose.pose.position.y;
    start_state[2] = tf::getYaw(msg.pose.pose.orientation);

    odom_flag = true;

    odom_counter = 0;
}

void DBPlanner::TargetCallback(const nav_msgs::Odometry& msg)
{
    final_state[0] = msg.pose.pose.position.x;
    final_state[1] = msg.pose.pose.position.y;
    final_state[2] = tf::getYaw(msg.pose.pose.orientation);

    target_flag = true;

    target_counter = 0;
}

void DBPlanner::LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    LaserCloudSurround->clear();
    pcl::fromROSMsg(*msg, *LaserCloudSurround);
    
    pass_z.setInputCloud (LaserCloudSurround);          // 设置输入点云
    pass_z.filter (*LaserCloudSurroundFiltered);        // 执行滤波
    pass_x.setInputCloud (LaserCloudSurroundFiltered);  // 设置输入点云
    pass_x.filter (*LaserCloudSurround);                // 执行滤波
    pass_y.setInputCloud (LaserCloudSurround);          // 设置输入点云
    pass_y.filter (*LaserCloudSurroundFiltered);        // 执行滤波
    
    downSizeFilter.setInputCloud(LaserCloudSurroundFiltered);
    downSizeFilter.filter(*LaserCloudSurround);

    // 生成地图
    for (size_t i = 0; i < map_rows; i++)
    {
        for (size_t j = 0; j < map_cols; j++)
            image_map.at<uchar>(i, j) = 255;
    }

    for (size_t i = 0; i < LaserCloudSurround->points.size(); i++)
    {
        pointSub = LaserCloudSurround->points[i];

        index_row = map_rows-1 - floor((pointSub.y - map_o_y)/map_res);
        index_col = floor((pointSub.x - map_o_x)/map_res);

        if (index_row >= 0 && index_row < map_rows && index_col >= 0 && index_col < map_cols)
            image_map.at<uchar>(index_row, index_col) = 0;
    }
    
    cv::distanceTransform(image_map, image_dis, CV_DIST_L2, 5, CV_32FC1);

    for (size_t i = 0; i < map_msg.info.height; i++)
    {
        for (size_t j = 0; j < map_msg.info.width; j++)
        {       
            // 图像上下颠倒
            if (image_map.at<uchar>(i, j) == 0)
                map_vec[(map_msg.info.height-1-i)*map_msg.info.width+j] = 100;
            else if (image_map.at<uchar>(i, j) == 255)
                map_vec[(map_msg.info.height-1-i)*map_msg.info.width+j] = 0;
        }
    }

    map_msg.data = map_vec;

    map_msg.header.stamp = ros::Time::now();

    MapPub.publish(map_msg);

    // 获取雷达边界点云
    GetLaserEdgePoint();

    lidar_flag = true;

    lidar_counter = 0;
}

void DBPlanner::GetLaserEdgePoint(void)
{
    for (size_t i = 0; i < laser_point.size(); i++)
    {
        laser_point[i].x_veh = Laser_Edge*cos(i*ANGLE_RES);

        laser_point[i].y_veh = Laser_Edge*sin(i*ANGLE_RES);

        laser_point[i].yaw_veh = atan2(laser_point[i].y_veh, laser_point[i].x_veh);
        
        laser_point[i].dis_veh = Laser_Edge;
    }

    for (size_t i = 0; i < LaserCloudSurround->points.size(); i++)
    {
        pointSub.x = LaserCloudSurround->points[i].x;
        pointSub.y = LaserCloudSurround->points[i].y;
        pointSub.z = LaserCloudSurround->points[i].z;

        horizon_angle = atan2(pointSub.y, pointSub.x);

        if (horizon_angle >= 0 && horizon_angle <= M_PI)
            horizon_index = floor(horizon_angle/ANGLE_RES);

        if (horizon_angle < 0 && horizon_angle > -M_PI)
            horizon_index = floor((2*M_PI + horizon_angle)/ANGLE_RES);
        
        dis_tmp = sqrt(pow(pointSub.x, 2) + pow(pointSub.y, 2));

        dis_min = sqrt(pow(laser_point[horizon_index].x_veh, 2) + 
                       pow(laser_point[horizon_index].y_veh, 2));        

        if (dis_tmp < dis_min)
        {
            laser_point[horizon_index].x_veh = pointSub.x;

            laser_point[horizon_index].y_veh = pointSub.y;

            laser_point[horizon_index].yaw_veh = atan2(pointSub.y, pointSub.x);

            laser_point[horizon_index].dis_veh = dis_tmp;
        }
    }
    
    // 寻找障碍边界点，并旋转采样
    laser_edge_point.clear();
    
    for (int i = 0; i < laser_point.size(); i++)
    {
        if (laser_point[i].dis_veh == Laser_Edge)
        {
            laser_edge_point.push_back(laser_point[i]);

            continue;
        }
        
        int k;

        neighbor_dis.clear();

        for (int j = i - Neighbor_num/2; j <= i + Neighbor_num/2; j++)
        {
            if (j == i)
                continue;
            
            k = j;
            
            if (k < 0)
                k += seg_num;
            
            if (k > seg_num-1)
                k -= seg_num;
                
            neighbor_dis.push_back(laser_point[k].dis_veh);
        }

        // 判断逆时针障碍边界
        for (size_t j = 0; j < neighbor_flag.size(); j++)
        {
            neighbor_flag[j] = false;
        }
        
        for (size_t j = 0; j < neighbor_dis.size(); j++)
        {
            if (j < Neighbor_num/2)
            {
                if (fabs(laser_point[i].dis_veh - neighbor_dis[j]) < Edge_tolerance)
                {
                    neighbor_flag[j] = true;
                }
            }
            else
            {
                if (fabs(laser_point[i].dis_veh - neighbor_dis[j]) >= Edge_tolerance)
                {
                    neighbor_flag[j] = true;
                }
            }
        }
        
        obs_edge_ccw_flag = neighbor_flag[0];

        if (obs_edge_ccw_flag)
        {
            for (size_t j = 1; j < neighbor_flag.size(); j++)
            {
                obs_edge_ccw_flag &= neighbor_flag[j];
            }
        }

        if (obs_edge_ccw_flag)
        {
            // theta_safe = (OBS_DIS)/laser_point[i].dis_veh;
            theta_safe = (k_safe*OBS_DIS)/laser_point[i].dis_veh;

            // for (float theta = theta_safe; theta <= k_safe*theta_safe; theta += ANGLE_RES)
            for (float theta = theta_safe; theta <= k_sample*theta_safe; theta += ANGLE_RES)        
            {
                laser_point_tmp.x_veh = laser_point[i].x_veh * cos(theta) - 
                                        laser_point[i].y_veh * sin(theta);

                laser_point_tmp.y_veh = laser_point[i].x_veh * sin(theta) + 
                                        laser_point[i].y_veh * cos(theta);
                
                laser_point_tmp.yaw_veh = atan2(laser_point_tmp.y_veh, laser_point_tmp.x_veh);

                laser_point_tmp.dis_veh = laser_point[i].dis_veh;

                laser_edge_point.push_back(laser_point_tmp);
            }
            
            continue;
        }
        
        // 判断顺时针障碍边界
        for (size_t j = 0; j < neighbor_flag.size(); j++)
        {
            neighbor_flag[j] = false;
        }

        for (size_t j = 0; j < neighbor_dis.size(); j++)
        {
            if (j < Neighbor_num/2)
            {
                if (fabs(laser_point[i].dis_veh - neighbor_dis[j]) >= Edge_tolerance)
                {
                    neighbor_flag[j] = true;
                }
            }
            else
            {
                if (fabs(laser_point[i].dis_veh - neighbor_dis[j]) < Edge_tolerance)
                {
                    neighbor_flag[j] = true;
                }
            }
        }

        obs_edge_cw_flag = neighbor_flag[0];

        if (obs_edge_cw_flag)
        {
            for (size_t j = 1; j < neighbor_flag.size(); j++)
            {
                obs_edge_cw_flag &= neighbor_flag[j];
            }
        }
        
        if (obs_edge_cw_flag)
        {
            // theta_safe = OBS_DIS/laser_point[i].dis_veh;
            theta_safe = (k_safe*OBS_DIS)/laser_point[i].dis_veh;            

            // for (float theta = theta_safe; theta <= k_safe*theta_safe; theta += ANGLE_RES)
            for (float theta = theta_safe; theta <= k_sample*theta_safe; theta += ANGLE_RES)
            {
                laser_point_tmp.x_veh = laser_point[i].x_veh * cos(-theta) - 
                                        laser_point[i].y_veh * sin(-theta);

                laser_point_tmp.y_veh = laser_point[i].x_veh * sin(-theta) + 
                                        laser_point[i].y_veh * cos(-theta);
                
                laser_point_tmp.yaw_veh = atan2(laser_point_tmp.y_veh, laser_point_tmp.x_veh);
                
                laser_point_tmp.dis_veh = laser_point[i].dis_veh;

                laser_edge_point.push_back(laser_point_tmp);
            }
        }
    }

    // std::cout << "laser_edge_point.size() = " << laser_edge_point.size() << std::endl;

    laser_edge_msg.poses.resize(laser_edge_point.size());
    
    for (size_t i = 0; i < laser_edge_msg.poses.size(); i++)
    {
        laser_edge_msg.poses[i].position.x = laser_edge_point[i].x_veh;
        laser_edge_msg.poses[i].position.y = laser_edge_point[i].y_veh;
        laser_edge_msg.poses[i].orientation = tf::createQuaternionMsgFromYaw(laser_edge_point[i].yaw_veh);
    }

    laser_edge_msg.header.stamp = ros::Time::now();
    
    LaserEdgePointPub.publish(laser_edge_msg);
    
    for (size_t i = 0; i < laser_point.size(); i++)
    {
        laser_point_msg.poses[i].position.x = laser_point[i].x_veh;
        laser_point_msg.poses[i].position.y = laser_point[i].y_veh;
        laser_point_msg.poses[i].orientation = tf::createQuaternionMsgFromYaw(laser_point[i].yaw_veh);
    }
    
    laser_point_msg.header.stamp = ros::Time::now();

    LaserPointPub.publish(laser_point_msg);
}