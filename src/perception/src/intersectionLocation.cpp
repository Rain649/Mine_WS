#include "intersectionLocation.h"
#include <ros/ros.h>

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

//将弧度转换到-π~π区间
inline void radianTransform(float &radian)
{
  while (radian > M_PI)
    radian -= 2 * M_PI;
  while (radian <= -M_PI)
    radian += 2 * M_PI;
}

int v1(1); //设置左窗口
int v2(2); //设置右窗口

void intersectionLocation(std::vector<float> &pose, const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::visualization::PCLVisualizer &viewer)
{
  /********读取数据********/
  std::string fin = "src/perception/include/ndtData.yaml";
  YAML::Node config = YAML::LoadFile(fin);
  float maximumIterations = config["maximumIterations"].as<float>();
  float resolution = config["resolution"].as<float>();
  float stepSize = config["stepSize"].as<float>();
  float transformationEpsilon = config["transformationEpsilon"].as<float>();
  float maxCorrespondenceDistance = config["maxCorrespondenceDistance"].as<float>();
  float euclideanFitnessEpsilon = config["euclideanFitnessEpsilon"].as<float>();

  float x_pre;
  float y_pre;
  float yaw_pre;
  /********测试数据********/
  if (config["menu_bool"].as<bool>())
  {
    x_pre = config["x"].as<float>();
    y_pre = config["y"].as<float>();
    yaw_pre = config["yaw"].as<float>() * M_PI / 180;
  }
  /********读取数据********/
  else
  {
    x_pre = pose[0];
    y_pre = pose[1];
    yaw_pre = pose[2];
  }
  radianTransform(yaw_pre);
  ROS_INFO("---------------------------------------");
  ROS_INFO_STREAM("yaw_pre =  " << yaw_pre << "; x =  " << x_pre << "; y =  " << y_pre);

  //将输入的扫描过滤到原始尺寸的大概10%以提高匹配的速度。
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);
  approximate_voxel_filter.setInputCloud(input_cloud);
  approximate_voxel_filter.filter(*filtered_cloud);
  // std::cout << "Filtered cloud contains " << filtered_cloud->size() << " data points from test2.pcd" << std::endl;
  pcl::PassThrough<pcl::PointXYZ> groundFilter;
  groundFilter.setInputCloud(target_cloud);
  groundFilter.setFilterFieldName("z");
  groundFilter.setFilterLimits(-0.8, 10);
  groundFilter.setFilterLimitsNegative(false);
  groundFilter.filter(*target_cloud);

  //创建ICP的实例类
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  //初始化正态分布变换（NDT）
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  // 匹配后的点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
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
      icp.align(*output_cloud, init_guess);
      Eigen::Matrix4f transformation = ndt.getFinalTransformation();
    }
#pragma omp section
    {
      //为终止条件设置最小转换差异
      ndt.setTransformationEpsilon(transformationEpsilon); //定义了[x,y,z,roll,pitch,yaw]在配准中的最小递增量，一旦递增量小于此限制，配准终止
      //为More-Thuente线搜索设置最大步长，步长越大迭代越快，但也容易导致错误
      ndt.setStepSize(stepSize);
      //设置NDT网格结构的分辨率（VoxelGridCovariance）
      ndt.setResolution(resolution); // ND体素的大小，单位为m,越小越准确，但占用内存越多
      //设置匹配迭代的最大次数
      ndt.setMaximumIterations(maximumIterations);
      // 设置要配准的点云
      ndt.setInputSource(filtered_cloud);
      //设置点云配准目标
      ndt.setInputTarget(target_cloud);
      //设置使用机器人测距法得到的初始对准估计结果
      Eigen::AngleAxisf init_rotation(yaw_pre, Eigen::Vector3f::UnitZ());
      Eigen::Translation3f init_translation(x_pre, y_pre, 0);
      Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
      ndt.align(*output_cloud, init_guess);
      Eigen::Matrix4f transformation = ndt.getFinalTransformation();
    }
  }
  Eigen::Matrix4f transformation;
  if (ndt.getFitnessScore() < icp.getFitnessScore())
  {
    transformation = ndt.getFinalTransformation();
    pcl::transformPointCloud(*filtered_cloud, *output_cloud, transformation);
    ROS_INFO_STREAM("NDT WIN, SCORE : " << ndt.getFitnessScore());
  }
  else
  {
    transformation = icp.getFinalTransformation();
    pcl::transformPointCloud(*filtered_cloud, *output_cloud, transformation);
    ROS_INFO_STREAM("ICP WIN, SCORE : " << icp.getFitnessScore());
  }

  //位姿更新
  if (std::min(icp.getFitnessScore(), ndt.getFitnessScore()) < 10)
  {
    pose[0] = transformation(0, 3);
    pose[1] = transformation(1, 3);
    pose[2] = (acos((transformation(0, 0) + transformation(1, 1)) / 2) + asin((-transformation(0, 1) + transformation(1, 0)) / 2)) / 2;
    radianTransform(pose[2]);
    ROS_INFO_STREAM("yaw =  " << pose[2] << "; x =  " << transformation(0, 3) << "; y =  " << transformation(1, 3));
  }

  //对目标点云着色（红色）并可视化
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
  //对转换前的输入点云着色（绿色）并可视化
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color(filtered_cloud, 0, 255, 0);
  //对转换后的输入点云着色（蓝色）并可视化
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(output_cloud, 0, 0, 255);

  viewer.removeAllPointClouds();

  viewer.addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud1", v1);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud1");
  viewer.addPointCloud<pcl::PointXYZ>(filtered_cloud, input_color, "input cloud", v1);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input cloud");

  //对目标点云着色（红色）并可视化
  viewer.addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud2", v2);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud2");
  // //对转换后的输入点云着色（蓝色）并可视化
  viewer.addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud", v2);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "output cloud");

  return;
}

//保存转换的输入点云
// pcl::io::savePCDFileASCII ("test_transformed.pcd", *output_cloud);
/////////////////////////////////////
// // 初始化点云可视化界面
// boost::shared_ptr<pcl::visualization::PCLVisualizer>
// viewer_origin (new pcl::visualization::PCLVisualizer ("Origin"));
// viewer_origin->setBackgroundColor (0, 0, 0);
// //对目标点云着色（红色）并可视化
// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
// target_color (output_cloud2, 255, 0, 0);
// viewer_origin->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
// viewer_origin->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                                 1, "target cloud");
// //对转换后的目标点云着色（绿色）并可视化
// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
// output_color (output_cloud, 0, 255, 0);
// viewer_origin->addPointCloud<pcl::PointXYZ> (input_cloud, output_color, "output cloud");
// viewer_origin->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                                 1, "output cloud");
// // 启动可视化
// viewer_origin->addCoordinateSystem (1.0);
// viewer_origin->initCameraParameters ();
//////////////////////////////////////////////////////

// // 初始化点云可视化界面
// boost::shared_ptr<pcl::visualization::PCLVisualizer>
// viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
// viewer_final->setBackgroundColor (0, 0, 0);
// //对目标点云着色（红色）并可视化
// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
// // target_color (output_cloud2, 255, 0, 0);
// viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud2, target_color, "target cloud");
// viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                                 1, "target cloud");
// //对转换后的目标点云着色（绿色）并可视化
// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
// // output_color (output_cloud, 0, 255, 0);
// viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
// viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                                 1, "output cloud");
// // 启动可视化
// viewer_final->addCoordinateSystem (1.0);
// viewer_final->initCameraParameters ();
//等待直到可视化窗口关闭。
// while (!viewer_final->wasStopped ())
// {
//   viewer_final->spinOnce (100);
//   viewer_origin->spinOnce (100);
//   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
// }