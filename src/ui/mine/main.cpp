#pragma execution_character_set("utf-8")

#include "mainwindow.h"
#include "controller.h"
#include "worker.h"
#include <QApplication>
#include <QTextCodec>

#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"ui");
    ROS_INFO("UI");

    QApplication a(argc, argv);
    //字体
    QFont font;
    font.setFamily("Microsoft Yahei");
    font.setPixelSize(13);
    a.setFont(font);

#if (QT_VERSION < QT_VERSION_CHECK(5,0,0))
#if _MSC_VER
    QTextCodec *codec = QTextCodec::codecForName("gbk");
#else
    QTextCodec *codec = QTextCodec::codecForName("utf-8");
#endif
    QTextCodec::setCodecForLocale(codec);
    QTextCodec::setCodecForCStrings(codec);
    QTextCodec::setCodecForTr(codec);
#else
    QTextCodec *codec = QTextCodec::codecForName("utf-8");
    QTextCodec::setCodecForLocale(codec);
#endif

    MainWindow *mw(new MainWindow);
    Controller c;
    c.setMainWindow(mw);
    c.start();
    mw->show();

    return a.exec();
}
