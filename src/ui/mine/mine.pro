QT += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
greaterThan(QT_MAJOR_VERSION, 5): QT += core5compat

TARGET      = mine
TEMPLATE    = app
DESTDIR     = $$PWD/../bin
CONFIG      += warn_off

SOURCES     += main.cpp \
    controller.cpp \
    worker.cpp
SOURCES     += mainwindow.cpp
SOURCES     += lightbutton.cpp

HEADERS     += mainwindow.h \
    controller.h \
    state.h \
    worker.h
HEADERS     += lightbutton.h

FORMS       += mainwindow.ui

INCLUDEPATH += /opt/ros/noetic/include
DEPENDPATH += /opt/ros/noetic/include
LIBS += -L/opt/ros/noetic/lib -lyaml-cpp -lroscpp -lroslib -lrosconsole -lroscpp_serialization -lrostime -lxmlrpcpp -lcpp_common -lrosconsole_log4cxx -lrosconsole_backend_interface
