#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

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

int main(int argc, char **argv)
{
  clock_t startTime, endTime;
  startTime = clock(); //计时开始
  /********读取数据********/
  std::string fin = "src/perception/include/ndtData.yaml";
  YAML::Node config = YAML::LoadFile(fin);
  float yaw = config["yaw"].as<float>() * M_PI / 180;
  float x = config["x"].as<float>();
  float y = config["y"].as<float>();
  float maximumIterations = config["maximumIterations"].as<float>();
  float resolution = config["resolution"].as<float>();
  float stepSize = config["stepSize"].as<float>();
  float transformationEpsilon = config["transformationEpsilon"].as<float>();
  std::string test1File = config["test1File"].as<std::string>();
  std::string test2File = config["test2File"].as<std::string>();
  float yaw_pre = config["yaw_pre"].as<float>() * M_PI / 180;
  float x_pre = config["x_pre"].as<float>();
  float y_pre = config["y_pre"].as<float>();
  float bol = config["bol"].as<bool>();

  /********读取数据********/

  //加载目标点云pcd
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(test1File, *target_cloud) == -1)
  {
    PCL_ERROR("Couldn't read file test1.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size() << " data points from room_scan1.pcd" << std::endl;
  //加载需要配准点云pcd
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(test2File, *input_cloud) == -1)
  {
    PCL_ERROR("Couldn't read file test2.pcd\n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;

  // static pcl::PointXYZ thisP;
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud_Trans(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud_Trans(new pcl::PointCloud<pcl::PointXYZ>);
  // // range filter,去除周围点云
  // range_filter(*input_cloud,*inputCloud_Trans,4,0.1);
  // range_filter(*target_cloud,*targetCloud_Trans,4,0.1);

  // inputCloud_Trans = input_cloud;
  for (const auto &p : *input_cloud)
  {
    pcl::PointXYZ thisP;
    float x1 = p.z + x;
    float y1 = p.x + y;
    float z1 = p.y;

    thisP.x = x1 * cos(yaw) + y1 * sin(yaw);
    thisP.y = -x1 * sin(yaw) + y1 * cos(yaw);
    thisP.z = z1;

    inputCloud_Trans->push_back(thisP);
  }
  for (const auto &p : *target_cloud)
  {
    pcl::PointXYZ thisP;
    float x1 = p.z;
    float y1 = p.x;
    float z1 = p.y;

    thisP.x = x1;
    thisP.y = y1;
    thisP.z = z1;

    targetCloud_Trans->push_back(thisP);
  }
  // targetCloud_Trans = target_cloud;

  //将输入的扫描过滤到原始尺寸的大概10%以提高匹配的速度。
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.09, 0.09, 0.09);
  approximate_voxel_filter.setInputCloud(inputCloud_Trans);
  approximate_voxel_filter.filter(*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size()
            << " data points from test2.pcd" << std::endl;

  // filtered_cloud = inputCloud_Trans;

  //初始化正态分布变换（NDT）
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  //设置依赖尺度NDT参数
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
  // ndt.setInputCloud(filtered_cloud);
  //设置点云配准目标
  ndt.setInputTarget(targetCloud_Trans);
  //设置使用机器人测距法得到的初始对准估计结果
  Eigen::AngleAxisf init_rotation(yaw_pre, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(x_pre, y_pre, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
  //计算需要的刚体变换以便将输入的点云匹配到目标点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align(*output_cloud, init_guess); //这一步是将降采样之后的点云经过变换后的到output_cloud
  std::cout << "Normal Distributions Transform has converged: " << ndt.hasConverged()
            << "; score: " << ndt.getFitnessScore() << std::endl; //欧式适合度评分
  //使用创建的变换对未过滤的输入点云进行变换
  // ndt.getFinalTransformation (）即最终变换
  Eigen::Matrix4f transformation = ndt.getFinalTransformation();
  endTime = clock(); //计时结束
  std::cout << "The run time is: " << (int)(endTime - startTime) / CLOCKS_PER_SEC << " s" << std::endl;
  std::cout << "Here is the matrix m:\n"
            << transformation << std::endl;
  pcl::transformPointCloud(*inputCloud_Trans, *output_cloud, transformation);
  Eigen::Matrix3f transformation_3f;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      transformation_3f(i, j) = transformation(i, j);

  Eigen::Vector3f eulerAngle = transformation_3f.eulerAngles(0, 1, 2);
  cout << "roll, pitch, yaw : " << eulerAngle[0] * 180 / M_PI << ", " << eulerAngle[1] * 180 / M_PI << ", " << eulerAngle[2] * 180 / M_PI << "." << endl;

  x_pre = transformation(0, 3);
  y_pre = transformation(1, 3);
  yaw_pre = eulerAngle[2];

  // 初始化点云可视化界面
  int v1(0); //设置左窗口
  int v2(1); //设置右窗口
  pcl::visualization::PCLVisualizer viewer("ndt");
  viewer.createViewPort(0.0, 0.0, 0.5, 1, v1);
  viewer.setBackgroundColor(0, 0, 0, v1);
  viewer.createViewPort(0.5, 0.0, 1, 1, v2);
  viewer.setBackgroundColor(0.5, 0.5, 0.5, v2);
  viewer.addCoordinateSystem(5);
  viewer.initCameraParameters();
  viewer.setCameraPosition(30, 40, 50, -3, -4, -5, 0);

  //对目标点云着色（红色）并可视化
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      target_color(targetCloud_Trans, 255, 0, 0);
  viewer.addPointCloud<pcl::PointXYZ>(targetCloud_Trans, target_color, "target cloud0", v1);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                          1, "target cloud0");
  //对转换后的目标点云着色（绿色）并可视化
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      output_color(inputCloud_Trans, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ>(inputCloud_Trans, output_color, "output cloud0", v1);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                          2, "output cloud0");

  //对目标点云着色（红色）并可视化
  viewer.addPointCloud<pcl::PointXYZ>(targetCloud_Trans, target_color, "target cloud", v2);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                          1, "target cloud");
  //对转换后的目标点云着色（绿色）并可视化
  viewer.addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud", v2);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                          2, "output cloud");
  // 启动可视化、等待直到可视化窗口关闭。
  while (!viewer.wasStopped())
  {
    viewer.spinOnce(10);
    boost::this_thread::sleep(boost::posix_time::seconds(0.5));
  }

  return 0;
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
// target_color (targetCloud_Trans, 255, 0, 0);
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
// // target_color (targetCloud_Trans, 255, 0, 0);
// viewer_final->addPointCloud<pcl::PointXYZ> (targetCloud_Trans, target_color, "target cloud");
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