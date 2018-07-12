/****************************************************************************
** Meta object code from reading C++ file 'pcsWindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../pcsWindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pcsWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_pcsWindow_t {
    QByteArrayData data[33];
    char stringdata0[421];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_pcsWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_pcsWindow_t qt_meta_stringdata_pcsWindow = {
    {
QT_MOC_LITERAL(0, 0, 9), // "pcsWindow"
QT_MOC_LITERAL(1, 10, 22), // "EntitySelectionChanged"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 8), // "uniqueID"
QT_MOC_LITERAL(4, 43, 24), // "EntitiesSelectionChanged"
QT_MOC_LITERAL(5, 68, 13), // "std::set<int>"
QT_MOC_LITERAL(6, 82, 5), // "entID"
QT_MOC_LITERAL(7, 88, 11), // "PointPicked"
QT_MOC_LITERAL(8, 100, 13), // "cloudUniqueID"
QT_MOC_LITERAL(9, 114, 10), // "pointIndex"
QT_MOC_LITERAL(10, 125, 1), // "x"
QT_MOC_LITERAL(11, 127, 1), // "y"
QT_MOC_LITERAL(12, 129, 15), // "CameraDisplaced"
QT_MOC_LITERAL(13, 145, 17), // "MouseWheelRotated"
QT_MOC_LITERAL(14, 163, 14), // "wheelDelta_deg"
QT_MOC_LITERAL(15, 178, 23), // "PerspectiveStateChanged"
QT_MOC_LITERAL(16, 202, 16), // "PixelSizeChanged"
QT_MOC_LITERAL(17, 219, 17), // "PivotPointChanged"
QT_MOC_LITERAL(18, 237, 11), // "pcsVector3D"
QT_MOC_LITERAL(19, 249, 16), // "CameraPosChanged"
QT_MOC_LITERAL(20, 266, 11), // "Translation"
QT_MOC_LITERAL(21, 278, 1), // "t"
QT_MOC_LITERAL(22, 280, 17), // "LeftButtonClicked"
QT_MOC_LITERAL(23, 298, 18), // "RightButtonClicked"
QT_MOC_LITERAL(24, 317, 10), // "MouseMoved"
QT_MOC_LITERAL(25, 328, 16), // "Qt::MouseButtons"
QT_MOC_LITERAL(26, 345, 14), // "ButtonReleased"
QT_MOC_LITERAL(27, 360, 9), // "Drawing3D"
QT_MOC_LITERAL(28, 370, 12), // "FilesDropped"
QT_MOC_LITERAL(29, 383, 9), // "filenames"
QT_MOC_LITERAL(30, 393, 10), // "ZoomGlobal"
QT_MOC_LITERAL(31, 404, 9), // "FrameRate"
QT_MOC_LITERAL(32, 414, 6) // "Redraw"

    },
    "pcsWindow\0EntitySelectionChanged\0\0"
    "uniqueID\0EntitiesSelectionChanged\0"
    "std::set<int>\0entID\0PointPicked\0"
    "cloudUniqueID\0pointIndex\0x\0y\0"
    "CameraDisplaced\0MouseWheelRotated\0"
    "wheelDelta_deg\0PerspectiveStateChanged\0"
    "PixelSizeChanged\0PivotPointChanged\0"
    "pcsVector3D\0CameraPosChanged\0Translation\0"
    "t\0LeftButtonClicked\0RightButtonClicked\0"
    "MouseMoved\0Qt::MouseButtons\0ButtonReleased\0"
    "Drawing3D\0FilesDropped\0filenames\0"
    "ZoomGlobal\0FrameRate\0Redraw"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_pcsWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      19,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      16,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  109,    2, 0x06 /* Public */,
       4,    1,  112,    2, 0x06 /* Public */,
       7,    4,  115,    2, 0x06 /* Public */,
      12,    2,  124,    2, 0x06 /* Public */,
      13,    1,  129,    2, 0x06 /* Public */,
      15,    0,  132,    2, 0x06 /* Public */,
      16,    1,  133,    2, 0x06 /* Public */,
      17,    1,  136,    2, 0x06 /* Public */,
      19,    1,  139,    2, 0x06 /* Public */,
      20,    1,  142,    2, 0x06 /* Public */,
      22,    2,  145,    2, 0x06 /* Public */,
      23,    2,  150,    2, 0x06 /* Public */,
      24,    3,  155,    2, 0x06 /* Public */,
      26,    0,  162,    2, 0x06 /* Public */,
      27,    0,  163,    2, 0x06 /* Public */,
      28,    1,  164,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      30,    0,  167,    2, 0x0a /* Public */,
      31,    0,  168,    2, 0x0a /* Public */,
      32,    0,  169,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, QMetaType::Int, QMetaType::UInt, QMetaType::Int, QMetaType::Int,    8,    9,   10,   11,
    QMetaType::Void, QMetaType::Float, QMetaType::Float,   10,   11,
    QMetaType::Void, QMetaType::Float,   14,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Float,    2,
    QMetaType::Void, 0x80000000 | 18,    2,
    QMetaType::Void, 0x80000000 | 18,    2,
    QMetaType::Void, 0x80000000 | 18,   21,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    2,    2,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    2,    2,
    QMetaType::Void, QMetaType::Int, QMetaType::Int, 0x80000000 | 25,    2,    2,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QStringList,   29,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void pcsWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        pcsWindow *_t = static_cast<pcsWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->EntitySelectionChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->EntitiesSelectionChanged((*reinterpret_cast< std::set<int>(*)>(_a[1]))); break;
        case 2: _t->PointPicked((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< uint(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        case 3: _t->CameraDisplaced((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 4: _t->MouseWheelRotated((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 5: _t->PerspectiveStateChanged(); break;
        case 6: _t->PixelSizeChanged((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 7: _t->PivotPointChanged((*reinterpret_cast< const pcsVector3D(*)>(_a[1]))); break;
        case 8: _t->CameraPosChanged((*reinterpret_cast< const pcsVector3D(*)>(_a[1]))); break;
        case 9: _t->Translation((*reinterpret_cast< const pcsVector3D(*)>(_a[1]))); break;
        case 10: _t->LeftButtonClicked((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 11: _t->RightButtonClicked((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 12: _t->MouseMoved((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< Qt::MouseButtons(*)>(_a[3]))); break;
        case 13: _t->ButtonReleased(); break;
        case 14: _t->Drawing3D(); break;
        case 15: _t->FilesDropped((*reinterpret_cast< const QStringList(*)>(_a[1]))); break;
        case 16: _t->ZoomGlobal(); break;
        case 17: _t->FrameRate(); break;
        case 18: _t->Redraw(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (pcsWindow::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::EntitySelectionChanged)) {
                *result = 0;
            }
        }
        {
            typedef void (pcsWindow::*_t)(std::set<int> );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::EntitiesSelectionChanged)) {
                *result = 1;
            }
        }
        {
            typedef void (pcsWindow::*_t)(int , unsigned  , int , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::PointPicked)) {
                *result = 2;
            }
        }
        {
            typedef void (pcsWindow::*_t)(float , float );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::CameraDisplaced)) {
                *result = 3;
            }
        }
        {
            typedef void (pcsWindow::*_t)(float );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::MouseWheelRotated)) {
                *result = 4;
            }
        }
        {
            typedef void (pcsWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::PerspectiveStateChanged)) {
                *result = 5;
            }
        }
        {
            typedef void (pcsWindow::*_t)(float );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::PixelSizeChanged)) {
                *result = 6;
            }
        }
        {
            typedef void (pcsWindow::*_t)(const pcsVector3D & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::PivotPointChanged)) {
                *result = 7;
            }
        }
        {
            typedef void (pcsWindow::*_t)(const pcsVector3D & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::CameraPosChanged)) {
                *result = 8;
            }
        }
        {
            typedef void (pcsWindow::*_t)(const pcsVector3D & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::Translation)) {
                *result = 9;
            }
        }
        {
            typedef void (pcsWindow::*_t)(int , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::LeftButtonClicked)) {
                *result = 10;
            }
        }
        {
            typedef void (pcsWindow::*_t)(int , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::RightButtonClicked)) {
                *result = 11;
            }
        }
        {
            typedef void (pcsWindow::*_t)(int , int , Qt::MouseButtons );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::MouseMoved)) {
                *result = 12;
            }
        }
        {
            typedef void (pcsWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::ButtonReleased)) {
                *result = 13;
            }
        }
        {
            typedef void (pcsWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::Drawing3D)) {
                *result = 14;
            }
        }
        {
            typedef void (pcsWindow::*_t)(const QStringList & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&pcsWindow::FilesDropped)) {
                *result = 15;
            }
        }
    }
}

const QMetaObject pcsWindow::staticMetaObject = {
    { &QVTKWidget::staticMetaObject, qt_meta_stringdata_pcsWindow.data,
      qt_meta_data_pcsWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *pcsWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *pcsWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_pcsWindow.stringdata0))
        return static_cast<void*>(const_cast< pcsWindow*>(this));
    return QVTKWidget::qt_metacast(_clname);
}

int pcsWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QVTKWidget::qt_metacall(_c, _id, _a);
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

// SIGNAL 0
void pcsWindow::EntitySelectionChanged(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void pcsWindow::EntitiesSelectionChanged(std::set<int> _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void pcsWindow::PointPicked(int _t1, unsigned  _t2, int _t3, int _t4)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void pcsWindow::CameraDisplaced(float _t1, float _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void pcsWindow::MouseWheelRotated(float _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void pcsWindow::PerspectiveStateChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 5, Q_NULLPTR);
}

// SIGNAL 6
void pcsWindow::PixelSizeChanged(float _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void pcsWindow::PivotPointChanged(const pcsVector3D & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void pcsWindow::CameraPosChanged(const pcsVector3D & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void pcsWindow::Translation(const pcsVector3D & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void pcsWindow::LeftButtonClicked(int _t1, int _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void pcsWindow::RightButtonClicked(int _t1, int _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void pcsWindow::MouseMoved(int _t1, int _t2, Qt::MouseButtons _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 12, _a);
}

// SIGNAL 13
void pcsWindow::ButtonReleased()
{
    QMetaObject::activate(this, &staticMetaObject, 13, Q_NULLPTR);
}

// SIGNAL 14
void pcsWindow::Drawing3D()
{
    QMetaObject::activate(this, &staticMetaObject, 14, Q_NULLPTR);
}

// SIGNAL 15
void pcsWindow::FilesDropped(const QStringList & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 15, _a);
}
QT_END_MOC_NAMESPACE
