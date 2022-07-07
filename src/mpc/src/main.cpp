#include <ros/ros.h>
#include <math.h>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <thread>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/Imu.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <dynamic_reconfigure/server.h>
#include <future>
#include <yaml-cpp/yaml.h>

#include "matrix2yaw.h"
#include "MPC.h"
#include "vehicleState.h"
#include "path_struct.h"
#include "visualization.h"
#include "polyfit.h"
#include "mpc_param.h"
#include "topoMap.h"
#include "mpc/mpc_Config.h"
#include "perception/mineServer.h"

class VehicleControl
{
private:
  ros::NodeHandle nh;

  VehicleState state;
  MPC _mpc;
  map<string, double> _mpc_params;
  double _ref_cte, _ref_etheta, _ref_vel, _min_vel, _max_angle, _max_throttle, _bound_value;
  int _w_cte, _w_etheta, _w_vel,
      _w_angle, _w_accel, _w_angle_d, _w_accel_d;
  TopoMap topoMap;

  int predictedTargetNum;
  int intersectionID_id;
  int _controller_freq;
  int _mpc_steps;
  double _dt;
  double expectedVelocity;
  double passDistance;
  double _offset;
  double _intervalDistance;
  double steer_value;
  bool islatencyCalculate;
  bool isLocation;
  bool pathReceived;
  bool intersectionVerified;
  bool newMsg;

  Eigen::VectorXd coeffsRightlane;
  Eigen::VectorXd coeffs;

  std::vector<int> pathPlanned;
  std::vector<Path_struct> globalPath_vec;

  tf::StampedTransform transform_v2w;

  ros::Subscriber state_sub;
  ros::Subscriber nodeIDIndex_sub;
  ros::Subscriber laneRight_sub;
  ros::Subscriber intersection_sub;
  ros::Subscriber isLocation_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber odom_sub;
  ros::Publisher marker_pub;
  ros::Publisher control_pub;
  ros::Publisher expectedSpeed_pub;
  ros::Publisher wheelAngle_pub;

  // 创建一个client，请求service
  ros::ServiceClient client;

public:
  VehicleControl() : nh("~")
  {
    intersectionID_id = 0;
    //加载参数
    std::string data_dir;
    nh.param<std::string>("simu_data_dir", data_dir, "src/common/simu_data/");
    ROS_INFO("simu_data path: %s", data_dir.c_str());

    // Parameter for MPC solver
    nh.param("mpc_steps", _mpc_steps, 50);
    nh.param("mpc_ref_vel", _ref_vel, 3.0);
    nh.param("mpc_min_vel", _min_vel, 3.0);
    nh.param("mpc_w_cte", _w_cte, 5000);
    nh.param("mpc_w_etheta", _w_etheta, 5000);
    nh.param("mpc_w_vel", _w_vel, 1);
    nh.param("mpc_w_angle", _w_angle, 100);
    nh.param("mpc_w_angle_d", _w_angle_d, 10000);
    nh.param("mpc_w_accel", _w_accel, 50);
    nh.param("mpc_w_accel_d", _w_accel_d, 10);
    // 固定参数
    nh.param("mpc_ref_cte", _ref_cte, 0.0);
    nh.param("mpc_ref_etheta", _ref_etheta, 0.0);
    nh.param("mpc_max_angle", _max_angle, 35 * M_PI / 180); // Maximal angle radian (~30 deg)
    nh.param("mpc_max_throttle", _max_throttle, 5.0);       // Maximal throttle accel
    nh.param("mpc_bound_value", _bound_value, 1.0e3);       // Bound value for other variables
    nh.param("controller_freq", _controller_freq, 20);
    nh.param("intervalDistance", _intervalDistance, 1.0);
    nh.param("offset", _offset, 3.0);

    if (_ref_vel < _min_vel)
    {
      _ref_vel = _min_vel;
      nh.setParam("/mpc/mpc_ref_vel_", _ref_vel);
    }

    _dt = double(1.0 / _controller_freq); // time step duration dt in s
    state = {0, 0, 0, 0, 0, 0};

    topoMap = loadMap(data_dir + "Vertex.yaml", data_dir + "Edge.yaml", data_dir);

    client = nh.serviceClient<perception::mineServer>("/service/node_ID_Manager");
    // 创建service消息
    perception::mineServer addsrv;
    addsrv.request.mode = 1;
    // 发布service请求，等待应答结果
    if (client.call(addsrv))
    {
      pathPlanned = addsrv.response.pathPlanned;
      printf("Path: ");
      for (auto i : pathPlanned)
        printf("%d -> ", i);
      printf("\n");
    }
    else
      ROS_ERROR("Failed to call service node_ID_Manager");

    newMsg = islatencyCalculate = pathReceived = isLocation = false;

    state_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &VehicleControl::stateHandler, this);
    laneRight_sub = nh.subscribe<std_msgs::Float32MultiArrayConstPtr>("/laneDetection/rightCoefficient", 1, &VehicleControl::laneRightHandler, this);
    nodeIDIndex_sub = nh.subscribe<std_msgs::Int32ConstPtr>("/navigation/intersectionID_id", 1, &VehicleControl::nodeIDHandler, this);
    intersection_sub = nh.subscribe<std_msgs::BoolConstPtr>("/intersectionDetection/intersectionVerified", 1, &VehicleControl::intersectionHandler, this);
    isLocation_sub = nh.subscribe<std_msgs::BoolConstPtr>("/navigation/isLocation", 1, &VehicleControl::isLocationHandler, this);
    imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu", 1, &VehicleControl::imuHandler, this);
    odom_sub = nh.subscribe<nav_msgs::OdometryConstPtr>("/navigation/intersectionOdom", 1, &VehicleControl::odomHandler, this);

    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    wheelAngle_pub = nh.advertise<std_msgs::Float32>("wheelAngle", 1);
    control_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd_mux/output", 1);
    expectedSpeed_pub = nh.advertise<std_msgs::Float32>("expectedSpeed", 1);
  }

  void getDynamicParameter()
  {
    ros::param::get("/mpc/predictedTargetNum_", predictedTargetNum);

    ros::param::get("/mpc/mpc_steps_", _mpc_steps);
    ros::param::get("/mpc/mpc_w_cte_", _w_cte);
    ros::param::get("/mpc/mpc_w_etheta_", _w_etheta);
    ros::param::get("/mpc/mpc_w_vel_", _w_vel);
    ros::param::get("/mpc/mpc_w_angle_", _w_angle);
    ros::param::get("/mpc/mpc_w_angle_d_", _w_angle_d);
    ros::param::get("/mpc/mpc_w_accel_", _w_accel);
    ros::param::get("/mpc/mpc_w_accel_d_", _w_accel_d);

    ros::param::get("/mpc/passDistance_", passDistance);
    ros::param::get("/mpc/mpc_ref_vel_", _ref_vel);
    ros::param::get("/mpc/mpc_min_vel_", _min_vel);

    ros::param::get("/mpc/islatencyCalculate_", islatencyCalculate);

    if (_ref_vel < _min_vel)
    {
      _ref_vel = _min_vel;
      nh.setParam("/mpc/mpc_ref_vel_", _ref_vel);
    }
  }

  // For converting back and forth between radians and degrees.
  double deg2rad(double x) { return x * M_PI / 180; }
  double rad2deg(double x) { return x * 180 / M_PI; }

  void tfReceive()
  {
    tf::TransformListener listener;
    while (ros::ok())
    {
      if (isLocation)
      {
        try
        {
          listener.lookupTransform("vehicle_base_link", "localMap",
                                   ros::Time(0), transform_v2w);
        }
        catch (tf::TransformException &ex)
        {
          ROS_ERROR("Error: %s", ex.what());
          // std::this_thread::sleep_for(chrono::seconds(1));
          continue;
        }
      }
      std::this_thread::sleep_for(chrono::milliseconds(20));
    }
    printf("TF Receive Exit.\n");
    return;
  }
  void stateHandler(const gazebo_msgs::ModelStates::ConstPtr &msg)
  {
    size_t index = 2;
    // state.x = msg->pose[index].position.x;
    // state.y = msg->pose[index].position.y;
    // Eigen::Quaterniond quaternion(msg->pose[index].orientation.w, msg->pose[index].orientation.x, msg->pose[index].orientation.y, msg->pose[index].orientation.z);
    // state.yaw = matrix2yaw(quaternion.toRotationMatrix());
    state.v = sqrt(pow(msg->twist[index].linear.x, 2) + pow(msg->twist[index].linear.y, 2));

    newMsg = true;
  }
  void odomHandler(const nav_msgs::OdometryConstPtr msg)
  {
    state.x = msg->pose.pose.position.x;
    state.y = msg->pose.pose.position.y;
    // state.v = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2));
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    state.yaw = yaw;

    newMsg = true;
  }
  void nodeIDHandler(const std_msgs::Int32ConstPtr msg)
  {
    intersectionID_id = msg->data;
  }
  void laneRightHandler(const std_msgs::Float32MultiArrayConstPtr msg)
  {
    size_t msgSize = msg->data.size();
    Eigen::VectorXd coeffTemp(msgSize);
    for (size_t i = 0; i < msgSize; ++i)
      coeffTemp[i] = static_cast<double>(msg->data[i]);
    coeffsRightlane = coeffTemp;

    newMsg = true;
  }
  void intersectionHandler(const std_msgs::BoolConstPtr msg)
  {
    intersectionVerified = msg->data;
  }
  void isLocationHandler(const std_msgs::BoolConstPtr msg)
  {
    isLocation = msg->data;
  }
  void imuHandler(const sensor_msgs::Imu::ConstPtr &msg)
  {
    // ROS_INFO("2");
  }
  std::vector<Path_struct> interpolation(std::vector<Path_struct> &input, const double intervalDistance)
  {
    std::vector<Path_struct> output;
    output.push_back(input[0]);
    for (size_t i = 1; i < input.size(); ++i)
    {
      size_t insertPointNum = sqrt(pow(input[i].x - input[i - 1].x, 2) + pow(input[i].y - input[i - 1].y, 2)) / intervalDistance;
      if (insertPointNum == 0)
        insertPointNum = 1;
      for (size_t j = 0; j < insertPointNum; ++j)
      {
        double x_temp = (input[i - 1].x * (insertPointNum - j) + input[i].x * j) / float(insertPointNum);
        double y_temp = (input[i - 1].y * (insertPointNum - j) + input[i].y * j) / float(insertPointNum);
        output.push_back(Path_struct(x_temp, y_temp));
      }
    }
    return output;
  }

  void pub()
  {
    std_msgs::Float32 float32_temp;
    float32_temp.data = expectedVelocity;
    expectedSpeed_pub.publish((float32_temp));

    float32_temp.data = steer_value * 180 / M_PI;
    wheelAngle_pub.publish((float32_temp));
  }

  void run()
  {
    while (ros::ok() && pathPlanned.empty())
      std::this_thread::sleep_for(std::chrono::seconds(1));

    while (ros::ok())
    {
      if (newMsg)
      {
        newMsg = false;
        if (intersectionVerified)
          intersectionControl();
        else
          tunnelControl();

        // //状态切换
        // ackermann_msgs::AckermannDriveStamped msg;
        // msg.header.stamp = ros::Time::now();
        // msg.header.frame_id = "vehicle_base_link";
        // msg.drive.speed = 0;
        // msg.drive.acceleration = 0;
        // msg.drive.jerk = 0;
        // msg.drive.steering_angle = 0;
        // msg.drive.steering_angle_velocity = 1;
        // control_pub.publish(msg);

        marker_pub.publish(showReferencePath(globalPath_vec));
        marker_pub.publish(showVehicle(state));
      }
    }
    printf("MPC Control Exit.\n");
    return;
  }
  void intersectionControl()
  {
    if (intersectionID_id == 0)
      return;

    std::vector<std::array<double, 3>>
        path_array = topoMap.get_path(pathPlanned[intersectionID_id - 1], pathPlanned[intersectionID_id], pathPlanned[intersectionID_id + 1]);
    globalPath_vec.clear();
    for (size_t i = 0; i < path_array.size(); ++i)
      globalPath_vec.push_back(Path_struct(path_array[i][0], path_array[i][1]));

    globalPath_vec = interpolation(globalPath_vec, _intervalDistance);
    size_t globalPathNum = globalPath_vec.size();

    int temp_id = intersectionID_id;
    //交叉路口
    while (ros::ok() && intersectionVerified && temp_id == intersectionID_id)
    {
      // ROS_INFO_THROTTLE(0.5, "Intersection Control");
      if (!isLocation)
      {
        tunnelControl();
      }
      else
      { // 考虑控制延迟，计算提前量
        if (islatencyCalculate)
        {
          state.x = state.x + (state.v) * cos(state.yaw) * latency;
          state.y = state.y + (state.v) * sin(state.yaw) * latency;
          state.yaw = state.yaw - state.v / L * state.delta * latency;
          state.v = state.v + state.acceleration * latency;
        }

        //坐标系转换
        pcl::PointCloud<pcl::PointXYZ> pc_temp, pc_trans;
        pc_temp.header.frame_id = "localMap";

        for (size_t i = 0; i < globalPathNum; ++i)
          pc_temp.push_back(pcl::PointXYZ{(float)globalPath_vec[i].x, (float)globalPath_vec[i].y, 0});

        pcl_ros::transformPointCloud(pc_temp, pc_trans, transform_v2w);
        std::vector<Path_struct> localPath_vec;
        std::vector<double> x_vec, y_vec;
        for (size_t i = 0; i < globalPathNum; ++i)
        {
          localPath_vec.push_back(Path_struct(pc_trans[i].x, pc_trans[i].y));
          x_vec.push_back(pc_trans[i].x);
          y_vec.push_back(pc_trans[i].y);
        }

        size_t closestIndex{0};
        for (size_t i = 0; i < globalPathNum; ++i)
        {
          if (x_vec[i] >= 0)
          {
            closestIndex = i;
            break;
          }
        }
        size_t endIndex{globalPathNum - 1};
        for (size_t i = closestIndex; i < globalPathNum; ++i)
        {
          if (x_vec[i] < x_vec[i - 1])
          {
            endIndex = i - 1;
            break;
          }
        }
        size_t targetHeadIndex{0};
        for (size_t i = closestIndex; i > 1; --i)
        {
          if (x_vec[i] < x_vec[i - 1])
          {
            targetHeadIndex = i;
            break;
          }
        }
        size_t num = endIndex - targetHeadIndex + 1;
        if (num < 4)
        {
          targetHeadIndex = 0;
          num = globalPathNum;
        }

        Eigen::Map<Eigen::VectorXd> ptsxeig(&x_vec[targetHeadIndex], num);
        Eigen::Map<Eigen::VectorXd> ptsyeig(&y_vec[targetHeadIndex], num);

        coeffs = polyfit(ptsxeig, ptsyeig, 3);
        // std::cout << "coeffs : " << coeffs << std::endl;

        std::vector<Path_struct> referencePoints;
        for (size_t i = targetHeadIndex; i < targetHeadIndex + num; ++i)
        {
          referencePoints.push_back(Path_struct(globalPath_vec[i].x, globalPath_vec[i].y));
        }
        marker_pub.publish(showReferencePoints(referencePoints));

        // calculate the cross track error
        double cte = polyeval(coeffs, 0);
        // calculate the orientation error
        // f(x) is the polynomial defining the reference trajectory
        // f'(x) = 3Ax^2 + 2Bx + C
        // double f_prime_x = 3*coeffs[3]*pow(px,2) + 2*coeffs[2]*px + coeffs[1];
        double f_prime_x = coeffs[1];
        double epsi = -atan(f_prime_x);
        // std::cout << "cte : " << cte << std::endl;
        // std::cout << "epsi : " << epsi << std::endl;

        // state(local)
        Eigen::VectorXd state_eig(6);
        state_eig << 0, 0, 0, state.v, cte, epsi;

        // Init parameters for MPC object
        _mpc_params["DT"] = _dt;
        //_mpc_params["LFR"] = _Lfr;
        _mpc_params["STEPS"] = static_cast<double>(_mpc_steps);
        _mpc_params["REF_CTE"] = _ref_cte;
        _mpc_params["REF_ETHETA"] = _ref_etheta;
        _mpc_params["REF_V"] = std::min(_ref_vel, 4.0);
        _mpc_params["MIN_V"] = std::min(_min_vel, 4.0);
        // _mpc_params["REF_V"] = _ref_vel;
        // _mpc_params["MIN_V"] = _min_vel;
        _mpc_params["W_CTE"] = static_cast<double>(_w_cte);
        _mpc_params["W_EPSI"] = static_cast<double>(_w_etheta);
        _mpc_params["W_V"] = static_cast<double>(_w_vel);
        _mpc_params["W_ANGLE"] = static_cast<double>(_w_angle);
        _mpc_params["W_A"] = static_cast<double>(_w_accel);
        _mpc_params["W_DANGLE"] = static_cast<double>(_w_angle_d);
        _mpc_params["W_DA"] = static_cast<double>(_w_accel_d);
        _mpc_params["ANGLE"] = _max_angle;
        _mpc_params["MAXTHR"] = _max_throttle;
        _mpc_params["BOUND"] = _bound_value;
        _mpc.LoadParams(_mpc_params);

        // solve mpc for state and reference trajectory
        // returns [steering_angle, acceleration,expectedVelocity]
        std::vector<double> actuations = _mpc.Solve(state_eig, coeffs);

        steer_value = actuations[0];
        double throttle_value = actuations[1];
        expectedVelocity = actuations[2];
        // ROS_INFO_THROTTLE(1,"steer, throttle : %f, %f", steer_value, throttle_value);

        // Display the MPC predicted trajectory by a Green line
        std::vector<Path_struct> predictedTrajectory;
        for (size_t i = 0; i < _mpc.mpc_x.size(); ++i)
          predictedTrajectory.push_back(Path_struct(_mpc.mpc_x[i], _mpc.mpc_y[i]));
        marker_pub.publish(showPredictedTrajectory(predictedTrajectory));

        // Display the polyLine
        int n_waypoints = 10;
        double step = 1.0;
        std::vector<Path_struct> polyLine;
        for (int i = -n_waypoints; i < n_waypoints; ++i)
          polyLine.push_back(Path_struct(step * i, polyeval(coeffs, step * i)));
        marker_pub.publish(showPolyLine(polyLine));

        ackermann_msgs::AckermannDriveStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "vehicle_base_link";
        msg.drive.speed = expectedVelocity / 5.58;
        msg.drive.acceleration = 0;
        msg.drive.jerk = 0;
        msg.drive.steering_angle = steer_value;
        msg.drive.steering_angle_velocity = 1;
        control_pub.publish(msg);

        marker_pub.publish(showReferencePath(globalPath_vec));
        marker_pub.publish(showVehicle(state));
      }
    }
    return;
  }
  void tunnelControl()
  {
    // ROS_INFO_THROTTLE(0.5, "Tunnel Control");
    while (ros::ok() && !isLocation)
    {
      // Init parameters for MPC object
      _mpc_params["DT"] = _dt;
      //_mpc_params["LFR"] = _Lfr;
      _mpc_params["STEPS"] = static_cast<double>(_mpc_steps);
      _mpc_params["REF_CTE"] = _ref_cte;
      _mpc_params["REF_ETHETA"] = _ref_etheta;
      if (intersectionVerified)
      {
        _mpc_params["REF_V"] = std::min(_ref_vel, 4.0);
        _mpc_params["MIN_V"] = std::min(_min_vel, 4.0);
      }
      else
      {
        _mpc_params["REF_V"] = _ref_vel;
        _mpc_params["MIN_V"] = _min_vel;
      }
      _mpc_params["W_CTE"] = static_cast<double>(_w_cte);
      _mpc_params["W_EPSI"] = static_cast<double>(_w_etheta);
      _mpc_params["W_V"] = static_cast<double>(_w_vel);
      _mpc_params["W_ANGLE"] = static_cast<double>(_w_angle);
      _mpc_params["W_A"] = static_cast<double>(_w_accel);
      _mpc_params["W_DANGLE"] = static_cast<double>(_w_angle_d);
      _mpc_params["W_DA"] = static_cast<double>(_w_accel_d);
      _mpc_params["ANGLE"] = _max_angle;
      _mpc_params["MAXTHR"] = _max_throttle;
      _mpc_params["BOUND"] = _bound_value;
      _mpc.LoadParams(_mpc_params);

      if (intersectionVerified && false)
      {
        coeffs = offsetFit(coeffsRightlane, _offset + 1);
      }
      else
      {
        coeffs = offsetFit(coeffsRightlane, _offset);
      }

      // calculate the cross track error
      double cte = polyeval(coeffs, 0);
      // calculate the orientation error
      // f(x) is the polynomial defining the reference trajectory
      // f'(x) = 3Ax^2 + 2Bx + C
      // double f_prime_x = 3*coeffs[3]*pow(px,2) + 2*coeffs[2]*px + coeffs[1];
      double f_prime_x = coeffs[1];
      double epsi = -atan(f_prime_x);

      // state(local)
      Eigen::VectorXd state_eig(6);
      state_eig << 0, 0, 0, state.v, cte, epsi;

      // solve mpc for state and reference trajectory
      // returns [steering_angle, acceleration,expectedVelocity]
      std::vector<double> actuations = _mpc.Solve(state_eig, coeffs);

      steer_value = actuations[0];
      double throttle_value = actuations[1];
      expectedVelocity = actuations[2];
      // ROS_INFO_THROTTLE(1, "expectedVelocity : %f", expectedVelocity);
      // ROS_INFO_THROTTLE(1,"steer, throttle : %f, %f", steer_value, throttle_value);

      // Display the MPC predicted trajectory by a Green line
      std::vector<Path_struct> predictedTrajectory;
      for (size_t i = 0; i < _mpc.mpc_x.size(); ++i)
        predictedTrajectory.push_back(Path_struct(_mpc.mpc_x[i], _mpc.mpc_y[i]));
      marker_pub.publish(showPredictedTrajectory(predictedTrajectory));

      // Display the polyLine
      size_t n_waypoints = 20;
      double step = 1.0;
      std::vector<Path_struct> polyLine;
      for (size_t i = 1; i < n_waypoints; ++i)
        polyLine.push_back(Path_struct(step * i, polyeval(coeffs, step * i)));
      marker_pub.publish(showPolyLine(polyLine));

      ackermann_msgs::AckermannDriveStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "vehicle_base_link";
      msg.drive.speed = expectedVelocity / 5.58;
      msg.drive.acceleration = 0;
      msg.drive.jerk = 0;
      msg.drive.steering_angle = steer_value;
      msg.drive.steering_angle_velocity = 1;
      control_pub.publish(msg);

      marker_pub.publish(showVehicle());
    }
    return;
  }
};

//动态调参
void callback(mpc::mpc_Config &config, uint32_t level)
{
  // ROS_INFO("Reconfigure Request: %d %d %d %d %d %d %f %f %s %s",
  //          config.segmentation_radius,
  //          config.width_threshold,
  //          config.distance_threshold,
  //          config.col_minus_threshold,
  //          config.median_size,
  //          config.cluster_size_min,
  //          config.cluster_radius,
  //          config.median_coefficient,
  //          config.bool_outlier_removal ? "True" : "False",
  //          config.bool_median_filter ? "True" : "False");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc");

  //动态参数调节
  dynamic_reconfigure::Server<mpc::mpc_Config>
      server;
  dynamic_reconfigure::Server<mpc::mpc_Config>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  VehicleControl control_obj;

  std::thread thread_tfReceive(&VehicleControl::tfReceive, &control_obj);
  std::thread thread_run(&VehicleControl::run, &control_obj);

  ros::Rate rate(50);
  while (ros::ok())
  {
    ros::spinOnce();

    control_obj.getDynamicParameter();

    control_obj.pub();

    rate.sleep();
  }

  thread_run.join();
  thread_tfReceive.join();

  printf("Exit Successfully.\n");

  return 0;
}
