#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <QObject>
#include<QThread>
#include<QDebug>
#include "mainwindow.h"
#include "worker.h"

// controller用于启动线程和处理线程执行结果
class Controller : public QObject
{
    Q_OBJECT
    QThread workerThread;

private:
    MainWindow* mw;
    Worker *worker;

public:
    Controller(QObject *parent= nullptr);
    void setMainWindow(MainWindow* mw);
    void start();
    ~Controller();

public slots:
    // 处理线程执行的结果
    void handleResults(const int rslt);
signals:
    // 发送信号触发线程
    void operate(const int);

};

#endif // CONTROLLER_H

