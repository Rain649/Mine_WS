#include "intersectionLocation.h"
#include <ros/ros.h>

// void range_filter(pcl::PointCloud<pcl::PointXYZ> & input_cloud, pcl::PointCloud<pcl::PointXYZ> & output_cloud,const float & min_scan_range_, const float min_z)
// {
//     double r;
//     pcl::PointXYZ p ;
//     for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = input_cloud.begin();
//         item != input_cloud.end(); ++item)
//     {
//         p.x = (double) item->x;
//         p.y = (double) item->y;
//         p.z = (double) item->z;
//         r = p.x * p.x + p.y * p.y;
//         if (r > min_scan_range_&&p.z >min_z) //filter the point distance lager than min_scan_range and height lower than min_z
//         {
//             output_cloud.push_back(p);
//         }
//     }
// }

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
  std::string fin = "/home/lsj/dev/Mine_WS/src/perception/include/ndtData.yaml";
  YAML::Node config = YAML::LoadFile(fin);
  float maximumIterations = config["maximumIterations"].as<float>();
  float resolution = config["resolution"].as<float>();
  float stepSize = config["stepSize"].as<float>();
  float transformationEpsilon = config["transformationEpsilon"].as<float>();
  float maxCorrespondenceDistance = config["maxCorrespondenceDistance"].as<float>();
  float euclideanFitnessEpsilon = config["euclideanFitnessEpsilon"].as<float>();
  //
  // float x_pre = config["x"].as<float>();
  // float y_pre = config["y"].as<float>();
  // float yaw_pre = config["yaw"].as<float>();
  /********读取数据********/
  float x_pre = pose[0];
  float y_pre = pose[1];
  float yaw_pre = pose[2];
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

  // icp配准
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //创建ICP的实例类
  icp.setInputSource(filtered_cloud);
  icp.setInputTarget(target_cloud);
  icp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
  icp.setTransformationEpsilon(transformationEpsilon);
  icp.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);
  icp.setMaximumIterations(maximumIterations);
  // icp.setRANSACIterations(0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //设置使用机器人测距法得到的初始对准估计结果
  Eigen::AngleAxisf init_rotation(yaw_pre, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(x_pre, y_pre, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
  icp.align(*output_cloud, init_guess);

  // if (icp.hasConverged() == false || icp.getFitnessScore() > 2)
  // {
  //   ROS_ERROR_STREAM("icp Failed : " << icp.getFitnessScore());
  // }

  //这一步是将降采样之后的点云经过变换后的到output_cloud
  std::cout << "ICP Transform has converged: " << icp.hasConverged() << "; score: " << icp.getFitnessScore() << std::endl; //欧式适合度评分
  //使用创建的变换对未过滤的输入点云进行变换
  Eigen::Matrix4f transformation = icp.getFinalTransformation();

  // // pcl::transformPointCloud(*filtered_cloud, *closed_cloud, icp.getFinalTransformation());
  // std::cout << "Here is the matrix m:\n"
  //           << transformation << std::endl;
  // Eigen::Matrix3f transformation_3f;
  // for (int i = 0; i < 3; ++i)
  //   for (int j = 0; j < 3; ++j)
  //     transformation_3f(i, j) = transformation(i, j);
  // Eigen::Vector3f eulerAngle = transformation_3f.eulerAngles(0, 1, 2);
  // std::cout << "roll, pitch, yaw : " << eulerAngle[0] * 180 / M_PI << ", " << eulerAngle[1] * 180 / M_PI << ", " << eulerAngle[2] * 180 / M_PI << "." << std::endl;

  pose[0] = transformation(0, 3);
  pose[1] = transformation(1, 3);
  // pose[2] = eulerAngle[2];
  pose[2] = (acos((transformation(0, 0) + transformation(1, 1)) / 2) + asin((-transformation(0, 1) + transformation(1, 0)) / 2)) / 2;
  radianTransform(pose[2]);
  ROS_INFO_STREAM("yaw =  " << pose[2] << "; x =  " << transformation(0, 3) << "; y =  " << transformation(1, 3));
  /////////////////////////////////////////////

  // //初始化正态分布变换（NDT）
  // pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  // //为终止条件设置最小转换差异
  // ndt.setTransformationEpsilon(transformationEpsilon); //定义了[x,y,z,roll,pitch,yaw]在配准中的最小递增量，一旦递增量小于此限制，配准终止
  // //为More-Thuente线搜索设置最大步长，步长越大迭代越快，但也容易导致错误
  // ndt.setStepSize(stepSize);
  // //设置NDT网格结构的分辨率（VoxelGridCovariance）
  // ndt.setResolution(resolution); // ND体素的大小，单位为m,越小越准确，但占用内存越多
  // //设置匹配迭代的最大次数
  // ndt.setMaximumIterations(maximumIterations);
  // // 设置要配准的点云
  // ndt.setInputSource(filtered_cloud);
  // //设置点云配准目标
  // ndt.setInputTarget(target_cloud);
  // //设置使用机器人测距法得到的初始对准估计结果
  // Eigen::AngleAxisf init_rotation(yaw_pre, Eigen::Vector3f::UnitZ());
  // Eigen::Translation3f init_translation(x_pre, y_pre, 0);
  // Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
  // //计算需要的刚体变换以便将输入的点云匹配到目标点云
  // pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // ndt.align(*output_cloud, init_guess);                                                                                                     //这一步是将降采样之后的点云经过变换后的到output_cloud
  // std::cout << "Normal Distributions Transform has converged: " << ndt.hasConverged() << "; score: " << ndt.getFitnessScore() << std::endl; //欧式适合度评分
  // //使用创建的变换对未过滤的输入点云进行变换
  // Eigen::Matrix4f transformation = ndt.getFinalTransformation();
  // std::cout << "Here is the matrix m:\n"
  //           << transformation << std::endl;
  // pcl::transformPointCloud(*filtered_cloud, *output_cloud, transformation);
  // Eigen::Matrix3f transformation_3f;
  // for (int i = 0; i < 3; ++i)
  //   for (int j = 0; j < 3; ++j)
  //     transformation_3f(i, j) = transformation(i, j);

  // Eigen::Vector3f eulerAngle = transformation_3f.eulerAngles(2, 0, 2);
  // std::cout << "roll, pitch, yaw : " << eulerAngle[0] * 180 / M_PI << ", " << eulerAngle[1] * 180 / M_PI << ", " << eulerAngle[2] * 180 / M_PI << "." << std::endl;

  // // pose[0] = transformation(0, 3);
  // // pose[1] = transformation(1, 3);
  // pose[2] = -eulerAngle[2];
  // radianTransform(pose[2]);
  // ROS_INFO_STREAM("yaw =  " << eulerAngle[2] << "; x =  " << transformation(0, 3) << "; y =  " << transformation(1, 3));

  // //初始化正态分布变换（NDT）
  // pcl::NormalDistributionsTransform<pcl::PointXY, pcl::PointXY> ndt;
  // //为终止条件设置最小转换差异
  // ndt.setTransformationEpsilon(transformationEpsilon); //定义了[x,y,z,roll,pitch,yaw]在配准中的最小递增量，一旦递增量小于此限制，配准终止
  // //为More-Thuente线搜索设置最大步长，步长越大迭代越快，但也容易导致错误
  // ndt.setStepSize(stepSize);
  // //设置NDT网格结构的分辨率（VoxelGridCovariance）
  // ndt.setResolution(resolution); // ND体素的大小，单位为m,越小越准确，但占用内存越多
  // //设置匹配迭代的最大次数
  // ndt.setMaximumIterations(maximumIterations);
  // // 设置要配准的点云
  // pcl::PointCloud<pcl::PointXY>::Ptr filtered_2D(new pcl::PointCloud<pcl::PointXY>);
  // for (auto i : *filtered_cloud)
  // {
  //   filtered_2D->push_back(pcl::PointXY(i.x, i.y));
  // }
  // ndt.setInputSource(filtered_2D);
  // //设置点云配准目标
  // pcl::PointCloud<pcl::PointXY>::Ptr target_2D(new pcl::PointCloud<pcl::PointXY>);
  // for (const auto i : *target_cloud)
  // {
  //   target_2D->push_back(pcl::PointXY(i.x, i.y));
  // }
  // ndt.setInputTarget(target_2D);
  // //设置使用机器人测距法得到的初始对准估计结果
  // Eigen::AngleAxisf init_rotation(yaw_pre, Eigen::Vector3f::UnitZ());
  // Eigen::Translation2f init_translation(x_pre, y_pre);
  // Eigen::Matrix3f init_guess = (init_translation * init_rotation).matrix();
  // //计算需要的刚体变换以便将输入的点云匹配到目标点云
  // pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXY>);
  // ndt.align(*output_cloud, init_guess);                                                                                                     //这一步是将降采样之后的点云经过变换后的到output_cloud
  // std::cout << "Normal Distributions Transform has converged: " << ndt.hasConverged() << "; score: " << ndt.getFitnessScore() << std::endl; //欧式适合度评分
  // //使用创建的变换对未过滤的输入点云进行变换
  // Eigen::Matrix4f transformation = ndt.getFinalTransformation();
  // std::cout << "Here is the matrix m:\n"
  //           << transformation << std::endl;
  // pcl::transformPointCloud(*filtered_cloud, *output_cloud, transformation);
  // Eigen::Matrix3f transformation_3f;
  // for (int i = 0; i < 3; ++i)
  //   for (int j = 0; j < 3; ++j)
  //     transformation_3f(i, j) = transformation(i, j);

  // Eigen::Vector3f eulerAngle = transformation_3f.eulerAngles(2, 0, 2);
  // std::cout << "roll, pitch, yaw : " << eulerAngle[0] * 180 / M_PI << ", " << eulerAngle[1] * 180 / M_PI << ", " << eulerAngle[2] * 180 / M_PI << "." << std::endl;

  // // pose[0] = transformation(0, 3);
  // // pose[1] = transformation(1, 3);
  // pose[2] = -eulerAngle[2];
  // radianTransform(pose[2]);
  // ROS_INFO_STREAM("yaw =  " << eulerAngle[2] << "; x =  " << transformation(0, 3) << "; y =  " << transformation(1, 3));

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