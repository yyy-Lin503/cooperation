/****************************************************************************
** Meta object code from reading C++ file 'radar_gui.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../radar_gui.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'radar_gui.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_RadarGUI_t {
    QByteArrayData data[21];
    char stringdata0[453];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_RadarGUI_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_RadarGUI_t qt_meta_stringdata_RadarGUI = {
    {
QT_MOC_LITERAL(0, 0, 8), // "RadarGUI"
QT_MOC_LITERAL(1, 9, 24), // "on_connectButton_clicked"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 26), // "on_configureButton_clicked"
QT_MOC_LITERAL(4, 62, 25), // "handleStartCaptureClicked"
QT_MOC_LITERAL(5, 88, 24), // "handleStopCaptureClicked"
QT_MOC_LITERAL(6, 113, 17), // "handleExitClicked"
QT_MOC_LITERAL(7, 131, 29), // "on_refreshPortsButton_clicked"
QT_MOC_LITERAL(8, 161, 23), // "on_pushButton_4_clicked"
QT_MOC_LITERAL(9, 185, 23), // "on_pushButton_6_clicked"
QT_MOC_LITERAL(10, 209, 23), // "updatePointCloudDisplay"
QT_MOC_LITERAL(11, 233, 14), // "processUdpData"
QT_MOC_LITERAL(12, 248, 19), // "on_action_triggered"
QT_MOC_LITERAL(13, 268, 21), // "on_action_3_triggered"
QT_MOC_LITERAL(14, 290, 21), // "on_action_5_triggered"
QT_MOC_LITERAL(15, 312, 21), // "on_action_4_triggered"
QT_MOC_LITERAL(16, 334, 19), // "handleZoomInClicked"
QT_MOC_LITERAL(17, 354, 20), // "handleZoomOutClicked"
QT_MOC_LITERAL(18, 375, 25), // "onClusterPlaybackFinished"
QT_MOC_LITERAL(19, 401, 31), // "on_offlineProcessButton_clicked"
QT_MOC_LITERAL(20, 433, 19) // "processOfflineFrame"

    },
    "RadarGUI\0on_connectButton_clicked\0\0"
    "on_configureButton_clicked\0"
    "handleStartCaptureClicked\0"
    "handleStopCaptureClicked\0handleExitClicked\0"
    "on_refreshPortsButton_clicked\0"
    "on_pushButton_4_clicked\0on_pushButton_6_clicked\0"
    "updatePointCloudDisplay\0processUdpData\0"
    "on_action_triggered\0on_action_3_triggered\0"
    "on_action_5_triggered\0on_action_4_triggered\0"
    "handleZoomInClicked\0handleZoomOutClicked\0"
    "onClusterPlaybackFinished\0"
    "on_offlineProcessButton_clicked\0"
    "processOfflineFrame"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RadarGUI[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      19,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  109,    2, 0x08 /* Private */,
       3,    0,  110,    2, 0x08 /* Private */,
       4,    0,  111,    2, 0x08 /* Private */,
       5,    0,  112,    2, 0x08 /* Private */,
       6,    0,  113,    2, 0x08 /* Private */,
       7,    0,  114,    2, 0x08 /* Private */,
       8,    0,  115,    2, 0x08 /* Private */,
       9,    0,  116,    2, 0x08 /* Private */,
      10,    0,  117,    2, 0x08 /* Private */,
      11,    0,  118,    2, 0x08 /* Private */,
      12,    0,  119,    2, 0x08 /* Private */,
      13,    0,  120,    2, 0x08 /* Private */,
      14,    0,  121,    2, 0x08 /* Private */,
      15,    0,  122,    2, 0x08 /* Private */,
      16,    0,  123,    2, 0x08 /* Private */,
      17,    0,  124,    2, 0x08 /* Private */,
      18,    0,  125,    2, 0x08 /* Private */,
      19,    0,  126,    2, 0x08 /* Private */,
      20,    0,  127,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void RadarGUI::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<RadarGUI *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->on_connectButton_clicked(); break;
        case 1: _t->on_configureButton_clicked(); break;
        case 2: _t->handleStartCaptureClicked(); break;
        case 3: _t->handleStopCaptureClicked(); break;
        case 4: _t->handleExitClicked(); break;
        case 5: _t->on_refreshPortsButton_clicked(); break;
        case 6: _t->on_pushButton_4_clicked(); break;
        case 7: _t->on_pushButton_6_clicked(); break;
        case 8: _t->updatePointCloudDisplay(); break;
        case 9: _t->processUdpData(); break;
        case 10: _t->on_action_triggered(); break;
        case 11: _t->on_action_3_triggered(); break;
        case 12: _t->on_action_5_triggered(); break;
        case 13: _t->on_action_4_triggered(); break;
        case 14: _t->handleZoomInClicked(); break;
        case 15: _t->handleZoomOutClicked(); break;
        case 16: _t->onClusterPlaybackFinished(); break;
        case 17: _t->on_offlineProcessButton_clicked(); break;
        case 18: _t->processOfflineFrame(); break;
        default: ;
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject RadarGUI::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_RadarGUI.data,
    qt_meta_data_RadarGUI,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *RadarGUI::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RadarGUI::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_RadarGUI.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int RadarGUI::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 19)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 19;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 19)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 19;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
