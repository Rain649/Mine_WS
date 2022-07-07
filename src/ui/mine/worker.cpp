#include "worker.h"

Worker::Worker(QObject *parent) : QObject(parent),nh("~")
{
    subIntersection = nh.subscribe<std_msgs::Bool>("/intersectionDetection/intersectionVerified", 1, &Worker::intersectionHandler, this);
    subDistance = nh.subscribe<std_msgs::Float32>("/navigation/distance", 1, &Worker::distanceHandler, this);
    subOdom = nh.subscribe<nav_msgs::Odometry>("/navigation/intersectionOdom", 1, &Worker::odomHandler, this);
    subIsLocation = nh.subscribe<std_msgs::Bool>("/navigation/isLocation", 1, &Worker::isLocationHandler, this);
    subBranchNum = nh.subscribe<std_msgs::Int32>("/intersectionDetection/peakNum", 1, &Worker::branchNumHandler, this);
    subExpectedSpeed = nh.subscribe<std_msgs::Float32>("/mpc/expectedSpeed", 1, &Worker::expectedSpeedHandler, this);
//    subRealSpeed = nh.subscribe<std_msgs::Float32>("/mpc/peakNum", 1, &Worker::expectedSpeedHandler, this);
    subState = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &Worker::realSpeedHandler, this);
    subSteeringAngleHandler = nh.subscribe<std_msgs::Float32>("/mpc/wheelAngle", 1, &Worker::steeringAngleHandler, this);
//    subPath = nh.subscribe<std_msgs::Int32MultiArray>("/pathplanned", 1, &Worker::pathHandler, this);
    subNodeIDIndex = nh.subscribe<std_msgs::Int32ConstPtr>("/navigation/intersectionID_id", 1, &Worker::nodeIDHandler, this);

    //共享内存
    smem = new QSharedMemory;
    smem->setKey("smy");
    if(smem->isAttached())//该进程已经被附加到共享内存段
    {
        if(!smem->detach())//从共享内存段中分离进程
            qDebug()<<"detach error";
        return;
    }

    if(!smem->create(sizeof(State),QSharedMemory::ReadWrite))//申请1024字节的内存
    {
        smem->attach(QSharedMemory::ReadWrite);
    }


    state = new State;
    state->distance = 0.0;
    state->speed = 0.0;
//    state->vertex_id = 0;
    state->branch_num = 0;
    state->isLocation = false;
    state->intersectionVerified = false;
    state->intersectionID_id = 0;
    //    state->pathPlanned={0};

    std::string fin = "/home/lsj/dev/IntersectionPerception_Planning_Control/src/common/map/PathPlanned.yaml";
    YAML::Node config = YAML::LoadFile(fin);
    YAML::Node pathPlanned_YN = config["pathPlanned"];
    for (YAML::const_iterator it = pathPlanned_YN.begin();it != pathPlanned_YN.end();++it)
    {
        state->pathPlanned.push_back(it->as<int>());
        std::cout << it->as<int>() << " > ";
    }
    std::cout << std::endl;

}

void Worker::setMainWindow(MainWindow* mw)
{
    this->mw = mw;
}

void Worker::doWork(int parameter)
{
    ros::Rate rate(50);

    while(ros::ok())
    {
        ++parameter;
        ros::spinOnce();

        smem->lock();//加锁
        State *to = (State*)(smem->data());
        memcpy(to,state,sizeof(State));//b把数据写入内存
        smem->unlock();//解锁

        rate.sleep();

        if(!mw->isVisible())
            break;
    }
    //发送结束信号
    emit resultReady(parameter);
}

void Worker::intersectionHandler(const std_msgs::Bool msg)
{
    intersectionVerified = msg.data;
    //    ROS_INFO("%d", intersectionVerified);
    state->intersectionVerified = intersectionVerified;
}

void Worker::distanceHandler(const std_msgs::Float32 msg)
{
    distance = msg.data;
    //    ROS_INFO("distance : %f", distance);

    state->distance = distance;
}
void Worker::odomHandler(const nav_msgs::Odometry msg)
{
    float v_x = msg.twist.twist.linear.x;
    float v_y = msg.twist.twist.linear.y;
    speed = 3.6*sqrt(pow(v_x, 2) + pow(v_y, 2));
    //    ROS_INFO("speed : %f", speed);

    state->speed = speed;
}

void Worker::isLocationHandler(const std_msgs::Bool msg)
{
    state->isLocation = msg.data;

}
void Worker::branchNumHandler(const std_msgs::Int32 msg)
{
    state->branch_num = msg.data;
}
void Worker::expectedSpeedHandler(const std_msgs::Float32 msg)
{
    state->expectedSpeed = msg.data;
}
void Worker::realSpeedHandler(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
  size_t index = 2;
  // state.x = msg->pose[index].position.x;
  // state.y = msg->pose[index].position.y;
  // Eigen::Quaterniond quaternion(msg->pose[index].orientation.w, msg->pose[index].orientation.x, msg->pose[index].orientation.y, msg->pose[index].orientation.z);
  // state.yaw = matrix2yaw(quaternion.toRotationMatrix());
  state->realSpeed = sqrt(pow(msg->twist[index].linear.x, 2) + pow(msg->twist[index].linear.y, 2));

}
void Worker::steeringAngleHandler(const std_msgs::Float32 msg)
{
    state->steeringAngle = msg.data;
}
void Worker::pathHandler(const std_msgs::Int32MultiArray msg)
{
    state->pathPlanned.clear();
    std::cout << "< Path";
    for (size_t i = 0; i < msg.data.size(); ++i)
    {
        std::cout << " > " << msg.data[i];
        state->pathPlanned.push_back(msg.data[i]);
    }
    std::cout << std::endl;
    subPath.shutdown();
}
void Worker::nodeIDHandler(const std_msgs::Int32ConstPtr msg)
{
  state->intersectionID_id = msg->data;
}

