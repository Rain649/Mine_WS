#ifndef WORKER_H
#define WORKER_H

#include <QObject>
#include<QDebug>
#include<QThread>
#include <QSharedMemory>

#include "ros/ros.h"
#include <thread>
#include <chrono>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <yaml-cpp/yaml.h>

#include<thread>

#include"state.h"
#include "mainwindow.h"

class Worker : public QObject
{
    Q_OBJECT
private:

    bool intersectionVerified;
    float distance;
    float speed;
    State *state;
    MainWindow* mw;

    ros::NodeHandle nh;
    ros::Subscriber subIntersection;
    ros::Subscriber subDistance;
    ros::Subscriber subOdom;
    ros::Subscriber subIsLocation;
    ros::Subscriber subBranchNum;
    ros::Subscriber subExpectedSpeed;
    ros::Subscriber subState;
    ros::Subscriber subSteeringAngleHandler;
    ros::Subscriber subPath;
    ros::Subscriber subNodeIDIndex;

    QSharedMemory *smem;
public:
    explicit Worker(QObject *parent = nullptr);

public slots:
     // doWork定义了线程要执行的操作
    void doWork(int parameter);
    void setMainWindow(MainWindow* mw);

    void intersectionHandler(const std_msgs::Bool msg);
    void isLocationHandler(const std_msgs::Bool msg);
    void branchNumHandler(const std_msgs::Int32 msg);
    void distanceHandler(const std_msgs::Float32 msg);
    void odomHandler(const nav_msgs::Odometry msg);
    void expectedSpeedHandler(const std_msgs::Float32 msg);
    void steeringAngleHandler(const std_msgs::Float32 msg);
    void realSpeedHandler(const gazebo_msgs::ModelStates::ConstPtr &msg);
    void pathHandler(const std_msgs::Int32MultiArray msg);
    void nodeIDHandler(const std_msgs::Int32ConstPtr msg);

// 线程完成工作时发送的信号
signals:
    void resultReady(const int result);

};

#endif // WORKER_H
