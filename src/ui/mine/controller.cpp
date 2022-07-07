#include "controller.h"

Controller::Controller(QObject *parent) : QObject(parent)
{
    worker = new Worker;
}

void Controller::handleResults(const int result)
{
    mw->close();
}

void Controller::setMainWindow(MainWindow* mw)
{
    this->mw = mw;
}
void Controller::start()
{
    worker->setMainWindow(mw);
    //调用moveToThread将该任务交给workThread
    worker->moveToThread(&workerThread);
    //operate信号发射后启动线程工作
    connect(this, SIGNAL(operate(const int)), worker, SLOT(doWork(int)));
    //该线程结束时销毁
    connect(&workerThread, &QThread::finished, worker, &QObject::deleteLater);
    //线程结束后发送信号，对结果进行处理
    connect(worker, SIGNAL(resultReady(int)), this, SLOT(handleResults(int)));
    //启动线程
    workerThread.start();
    //发射信号，开始执行
    emit operate(0);
}

//析构函数中调用quit()函数结束线程
Controller::~Controller()
{
    workerThread.quit();
    workerThread.wait();
}
