/****************************************************************************
** Meta object code from reading C++ file 'worker.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.12)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../mine/worker.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'worker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.12. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Worker_t {
    QByteArrayData data[27];
    char stringdata0[398];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Worker_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Worker_t qt_meta_stringdata_Worker = {
    {
QT_MOC_LITERAL(0, 0, 6), // "Worker"
QT_MOC_LITERAL(1, 7, 11), // "resultReady"
QT_MOC_LITERAL(2, 19, 0), // ""
QT_MOC_LITERAL(3, 20, 6), // "result"
QT_MOC_LITERAL(4, 27, 6), // "doWork"
QT_MOC_LITERAL(5, 34, 9), // "parameter"
QT_MOC_LITERAL(6, 44, 13), // "setMainWindow"
QT_MOC_LITERAL(7, 58, 11), // "MainWindow*"
QT_MOC_LITERAL(8, 70, 2), // "mw"
QT_MOC_LITERAL(9, 73, 19), // "intersectionHandler"
QT_MOC_LITERAL(10, 93, 14), // "std_msgs::Bool"
QT_MOC_LITERAL(11, 108, 3), // "msg"
QT_MOC_LITERAL(12, 112, 17), // "isLocationHandler"
QT_MOC_LITERAL(13, 130, 16), // "branchNumHandler"
QT_MOC_LITERAL(14, 147, 15), // "std_msgs::Int32"
QT_MOC_LITERAL(15, 163, 15), // "distanceHandler"
QT_MOC_LITERAL(16, 179, 17), // "std_msgs::Float32"
QT_MOC_LITERAL(17, 197, 11), // "odomHandler"
QT_MOC_LITERAL(18, 209, 18), // "nav_msgs::Odometry"
QT_MOC_LITERAL(19, 228, 20), // "expectedSpeedHandler"
QT_MOC_LITERAL(20, 249, 20), // "steeringAngleHandler"
QT_MOC_LITERAL(21, 270, 16), // "realSpeedHandler"
QT_MOC_LITERAL(22, 287, 34), // "gazebo_msgs::ModelStates::Con..."
QT_MOC_LITERAL(23, 322, 11), // "pathHandler"
QT_MOC_LITERAL(24, 334, 25), // "std_msgs::Int32MultiArray"
QT_MOC_LITERAL(25, 360, 13), // "nodeIDHandler"
QT_MOC_LITERAL(26, 374, 23) // "std_msgs::Int32ConstPtr"

    },
    "Worker\0resultReady\0\0result\0doWork\0"
    "parameter\0setMainWindow\0MainWindow*\0"
    "mw\0intersectionHandler\0std_msgs::Bool\0"
    "msg\0isLocationHandler\0branchNumHandler\0"
    "std_msgs::Int32\0distanceHandler\0"
    "std_msgs::Float32\0odomHandler\0"
    "nav_msgs::Odometry\0expectedSpeedHandler\0"
    "steeringAngleHandler\0realSpeedHandler\0"
    "gazebo_msgs::ModelStates::ConstPtr\0"
    "pathHandler\0std_msgs::Int32MultiArray\0"
    "nodeIDHandler\0std_msgs::Int32ConstPtr"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Worker[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   79,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    1,   82,    2, 0x0a /* Public */,
       6,    1,   85,    2, 0x0a /* Public */,
       9,    1,   88,    2, 0x0a /* Public */,
      12,    1,   91,    2, 0x0a /* Public */,
      13,    1,   94,    2, 0x0a /* Public */,
      15,    1,   97,    2, 0x0a /* Public */,
      17,    1,  100,    2, 0x0a /* Public */,
      19,    1,  103,    2, 0x0a /* Public */,
      20,    1,  106,    2, 0x0a /* Public */,
      21,    1,  109,    2, 0x0a /* Public */,
      23,    1,  112,    2, 0x0a /* Public */,
      25,    1,  115,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, 0x80000000 | 14,   11,
    QMetaType::Void, 0x80000000 | 16,   11,
    QMetaType::Void, 0x80000000 | 18,   11,
    QMetaType::Void, 0x80000000 | 16,   11,
    QMetaType::Void, 0x80000000 | 16,   11,
    QMetaType::Void, 0x80000000 | 22,   11,
    QMetaType::Void, 0x80000000 | 24,   11,
    QMetaType::Void, 0x80000000 | 26,   11,

       0        // eod
};

void Worker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Worker *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->resultReady((*reinterpret_cast< const int(*)>(_a[1]))); break;
        case 1: _t->doWork((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->setMainWindow((*reinterpret_cast< MainWindow*(*)>(_a[1]))); break;
        case 3: _t->intersectionHandler((*reinterpret_cast< const std_msgs::Bool(*)>(_a[1]))); break;
        case 4: _t->isLocationHandler((*reinterpret_cast< const std_msgs::Bool(*)>(_a[1]))); break;
        case 5: _t->branchNumHandler((*reinterpret_cast< const std_msgs::Int32(*)>(_a[1]))); break;
        case 6: _t->distanceHandler((*reinterpret_cast< const std_msgs::Float32(*)>(_a[1]))); break;
        case 7: _t->odomHandler((*reinterpret_cast< const nav_msgs::Odometry(*)>(_a[1]))); break;
        case 8: _t->expectedSpeedHandler((*reinterpret_cast< const std_msgs::Float32(*)>(_a[1]))); break;
        case 9: _t->steeringAngleHandler((*reinterpret_cast< const std_msgs::Float32(*)>(_a[1]))); break;
        case 10: _t->realSpeedHandler((*reinterpret_cast< const gazebo_msgs::ModelStates::ConstPtr(*)>(_a[1]))); break;
        case 11: _t->pathHandler((*reinterpret_cast< const std_msgs::Int32MultiArray(*)>(_a[1]))); break;
        case 12: _t->nodeIDHandler((*reinterpret_cast< const std_msgs::Int32ConstPtr(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 2:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< MainWindow* >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Worker::*)(const int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Worker::resultReady)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Worker::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_Worker.data,
    qt_meta_data_Worker,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Worker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Worker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Worker.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int Worker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void Worker::resultReady(const int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
