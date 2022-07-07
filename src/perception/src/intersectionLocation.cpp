#include "intersectionLocation.h"

// // 初始化点云可视化界面
// int v1(0); //设置左窗口
// int v2(1); //设置右窗口
// viewer.createViewPort(0.0, 0.0, 0.5, 1, v1);
// viewer.setBackgroundColor(0, 0, 0, v1);
// viewer.createViewPort(0.5, 0.0, 1, 1, v2);
// viewer.setBackgroundColor(0.5, 0.5, 0.5, v2);
// viewer.addCoordinateSystem(5);
// viewer.initCameraParameters();
// viewer.setCameraPosition(30, 40, 50, -3, -4, -5, 0);

ros::Time lastLidarStamp;
ros::Time currentLidarStamp;

int locationTimes{0};
// int error_place{0};

inline void
radianTransform(float &radian)
{
  while (radian > M_PI)
    radian -= 2 * M_PI;
  while (radian <= -M_PI)
    radian += 2 * M_PI;
}

int v1(1); //设置左窗口
int v2(2); //设置右窗口

bool intersectionLocation(VehicleState &vehicleState, const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, RegistrationConfig &registrationConfig, pcl::visualization::PCLVisualizer::Ptr viewer)
{
  // error_place = 0;
  if (input_cloud->empty())
    return false;
  currentLidarStamp = pcl_conversions::fromPCL(input_cloud->header.stamp);
  // pcl_convers

  /********读取数据********/
  float maximumIterations = registrationConfig.maximumIterations;
  float resolution = registrationConfig.resolution;
  float stepSize = registrationConfig.stepSize;
  float transformationEpsilon = registrationConfig.transformationEpsilon;
  float maxCorrespondenceDistance = registrationConfig.maxCorrespondenceDistance;
  float euclideanFitnessEpsilon = registrationConfig.euclideanFitnessEpsilon;
  double yaw_thre = registrationConfig.yaw_thre;
  double fitnessScore_thre = registrationConfig.fitnessScore_thre;
  bool menu_bool = registrationConfig.menu_bool;

  float x_pre;
  float y_pre;
  float yaw_pre;
  /********测试数据********/
  if (menu_bool)
  {
    x_pre = registrationConfig.x_pre;
    y_pre = registrationConfig.y_pre;
    yaw_pre = registrationConfig.yaw_pre;
  }
  /********读取数据********/
  else
  {
    x_pre = vehicleState.x;
    y_pre = vehicleState.y;
    yaw_pre = vehicleState.yaw;
  }
  radianTransform(yaw_pre);
  // ROS_INFO("---------------------------------------");

  //将输入的扫描过滤到原始尺寸的大概10%以提高匹配的速度。
  pcl::PointCloud<pcl::PointXYZ> cloudTemp;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.3, 0.3, 0.3);
  approximate_voxel_filter.setInputCloud(input_cloud);
  approximate_voxel_filter.filter(cloudTemp);

  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(cloudTemp, *filtered_cloud, mapping);

  if (filtered_cloud->empty())
  {
    return false;
  }

  //创建ICP的实例类
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  //初始化正态分布变换（NDT）
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  if (filtered_cloud->empty())
  {
    return false;
  }

#pragma omp parallel sections
  {
#pragma omp section
    {
      // icp配准
      icp.setInputSource(filtered_cloud);
      icp.setInputTarget(target_cloud);
      icp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
      icp.setTransformationEpsilon(transformationEpsilon);
      icp.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);
      icp.setMaximumIterations(maximumIterations);
      //设置使用机器人测距法得到的初始对准估计结果
      Eigen::AngleAxisf init_rotation(yaw_pre, Eigen::Vector3f::UnitZ());
      Eigen::Translation3f init_translation(x_pre, y_pre, 0);
      Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
      // 匹配后的点云
      pcl::PointCloud<pcl::PointXYZ> cloud_temp;
      icp.align(cloud_temp, init_guess);
    }
#pragma omp section
    {
      // ndt配准
      //为终止条件设置最小转换差异
      ndt.setTransformationEpsilon(transformationEpsilon); //定义了[x,y,z,roll,pitch,yaw]在配准中的最小递增量，一旦递增量小于此限制，配准终止
      //为More-Thuente线搜索设置最大步长，步长越大迭代越快，但也容易导致错误
      ndt.setStepSize(stepSize);
      //设置NDT网格结构的分辨率（VoxelGridCovariance）
      ndt.setResolution(resolution); // ND体素的大小，单位为m,越小越准确，但占用内存越多
      //设置匹配迭代的最大次数
      ndt.setMaximumIterations(maximumIterations);
      // 设置要配准的点云
      // ndt.setInputSource(input_cloud);
      ndt.setInputSource(filtered_cloud);
      //设置点云配准目标
      ndt.setInputTarget(target_cloud);
      //设置使用机器人测距法得到的初始对准估计结果
      Eigen::AngleAxisf init_rotation(yaw_pre, Eigen::Vector3f::UnitZ());
      Eigen::Translation3f init_translation(x_pre, y_pre, 0);
      Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
      // 匹配后的点云
      pcl::PointCloud<pcl::PointXYZ> cloud_temp;
      ndt.align(cloud_temp, init_guess);
    }
  }

  // 匹配后的点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (std::min(icp.getFitnessScore(), ndt.getFitnessScore()) <= fitnessScore_thre)
  {
    Eigen::Matrix4f transformation;
    if (ndt.getFitnessScore() < icp.getFitnessScore())
    {
      transformation = ndt.getFinalTransformation();
      pcl::transformPointCloud(*filtered_cloud, *output_cloud, transformation);
      // ROS_INFO_STREAM("NDT WIN, SCORE : " << ndt.getFitnessScore());
    }
    else
    {
      transformation = icp.getFinalTransformation();
      pcl::transformPointCloud(*filtered_cloud, *output_cloud, transformation);
      // ROS_INFO_STREAM("ICP WIN, SCORE : " << icp.getFitnessScore());
    }
    double yaw_sin = (-transformation(0, 1) + transformation(1, 0)) / 2;
    double yaw_cos = (transformation(0, 0) + transformation(1, 1)) / 2;
    double yaw_registration;
    if (yaw_cos == 0)
      yaw_registration = (yaw_sin > 0) ? M_PI / 2 : -M_PI / 2;
    else
    {
      yaw_registration = atan(yaw_sin / yaw_cos);
      if (yaw_cos < 0)
        yaw_registration += (yaw_sin > 0) ? M_PI : -M_PI;
    }

    //异常判断
    float yaw_diff = abs(vehicleState.yaw - yaw_registration);
    if (yaw_diff > yaw_thre * M_PI / 180)
    {
      if (2 * M_PI - yaw_diff > yaw_thre * M_PI / 180)
      {
        // ROS_INFO("yaw change too big: %f from %f to %f ", yaw_diff * 180 / M_PI, vehicleState.yaw * 180 / M_PI, yaw_registration * 180 / M_PI);
        return false;
      }
    }

    pcl::PointXY p = {vehicleState.x, vehicleState.y};
    pcl::PointXY t = {transformation(0, 3), transformation(1, 3)};
    float distance = pcl::euclideanDistance(p, t);
    if (distance > 8)
    {
      // ROS_INFO("distance change too big: %f", distance);
      return false;
    }
    float timeInterval;
    if (locationTimes == 0)
      timeInterval = 0.1;
    else
      timeInterval = (currentLidarStamp - lastLidarStamp).toSec();
    float velocity = sqrt(pow(transformation(0, 3) - vehicleState.x, 2) + pow(transformation(1, 3) - vehicleState.y, 2)) / timeInterval;
    // if (locationTimes > 5)
    // {
    //   if (timeInterval > 0.5)
    //   {
    //     ROS_INFO("time interval too big: %f", timeInterval);
    //     return false;
    //   }
    //   if (velocity - vehicleState.v < -5)
    //   {
    //     ROS_INFO("velocity too small: %f", velocity);
    //     return false;
    //   }
    // }
    // float acceleration = abs(velocity - vehicleState.v) / timeInterval;
    // if (locationTimes > 5 && acceleration > 8)
    // {
    //   ROS_INFO("%f : %f", vehicleState.v, velocity);
    //   ROS_INFO("acceleration too big: %f", acceleration);
    //   return false;
    // }

    //位姿更新
    vehicleState.v = velocity;
    vehicleState.x = transformation(0, 3);
    vehicleState.y = transformation(1, 3);
    vehicleState.yaw = yaw_registration;
    radianTransform(vehicleState.yaw);
    lastLidarStamp = currentLidarStamp;
    // ROS_INFO_STREAM("yaw =  " << vehicleState.yaw << "; x =  " << transformation(0, 3) << "; y =  " << transformation(1, 3));

    if (viewer != nullptr)
    {
      //对目标点云着色（红色）并可视化
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
      //对转换前的输入点云着色（绿色）并可视化
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color(filtered_cloud, 0, 255, 0);
      //对转换后的输入点云着色（蓝色）并可视化
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(output_cloud, 0, 0, 255);

      viewer->removeAllPointClouds();

      viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud1", v1);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud1");
      viewer->addPointCloud<pcl::PointXYZ>(filtered_cloud, input_color, "input cloud", v1);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input cloud");

      //对目标点云着色（红色）并可视化
      viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud2", v2);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud2");
      //对转换后的输入点云着色（蓝色）并可视化
      viewer->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud", v2);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "output cloud");
    }
  }
  else
  {
    // ROS_ERROR_STREAM_THROTTLE(0.5, "FitnessScore Too Big:  " << std::min(icp.getFitnessScore(), ndt.getFitnessScore()));
    return false;
  }
  ++locationTimes;
  // ROS_INFO("times: %d", locationTimes);
  return true;
}
