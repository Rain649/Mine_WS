#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "qtimer.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("地下矿无人驾驶");

    smem = new QSharedMemory;
    smem->setKey("smy");//设置暗号,相当于钥匙,这个钥匙才能代开共享内存的锁

    initForm();
}

MainWindow::~MainWindow()
{
    delete ui;
    delete smem;
}

void MainWindow::initForm()
{
    type = 0;

    ui->frame->setFrameStyle(QFrame::Box);
    ui->stateLight_1->setBgColor(QColor(255,107,107));
    ui->lcdNumber_1->setDigitCount(1);
    ui->lcdNumber_1->setDecMode();
    ui->lcdNumber_2->setDigitCount(5);
    ui->lcdNumber_2->setSmallDecimalPoint(true);
    ui->lcdNumber_2->setDecMode();
    ui->lcdNumber_3->setDigitCount(5);
    ui->lcdNumber_3->setSmallDecimalPoint(true);
    ui->lcdNumber_3->setDecMode();
    ui->lcdNumber_4->setDigitCount(5);
    ui->lcdNumber_4->setSmallDecimalPoint(true);
    ui->lcdNumber_4->setDecMode();
    ui->lcdNumber_5->setDigitCount(5);
    ui->lcdNumber_5->setSmallDecimalPoint(true);
    ui->lcdNumber_5->setDecMode();
    ui->lcdNumber_6->setDigitCount(5);
    ui->lcdNumber_6->setSmallDecimalPoint(true);
    ui->lcdNumber_6->setDecMode();
    ui->lcdNumber_7->setDigitCount(5);
    ui->lcdNumber_7->setSmallDecimalPoint(true);
    ui->lcdNumber_7->setDecMode();


    QTimer *timer = new QTimer(this);
    timer->setInterval(500);
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeOut()));
    timer->start();
}

void MainWindow::onTimeOut()
{
    if(!smem->attach(QSharedMemory::ReadOnly))
    {
        qDebug()<<"mainWindow attach error!!!";
    }
    smem->lock();

    State *state;
    state = (State *)smem->constData();

    // 获取系统当前时间
    //    QDateTime dateTime = QDateTime::currentDateTime();

    if (state->intersectionVerified)
        ui->stateLight_1->setLightGreen();
    else
        ui->stateLight_1->setLightRed();

    if (state->isLocation)
        ui->stateLight_2->setLightGreen();
    else
        ui->stateLight_2->setLightRed();

    // 显示的内容
    ui->lcdNumber_1->display(state->branch_num);
    ui->lcdNumber_2->display(state->distance);
    ui->lcdNumber_3->display(state->speed);
    ui->lcdNumber_4->display(state->expectedSpeed);
    ui->lcdNumber_5->display(state->realSpeed);
    ui->lcdNumber_6->display(state->steeringAngle);
    ui->lcdNumber_7->display(state->pathPlanned[state->intersectionID_id]);

    smem->unlock();
    smem->detach();
}
