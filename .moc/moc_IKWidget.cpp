/****************************************************************************
** Meta object code from reading C++ file 'IKWidget.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.6.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/IKWidget.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'IKWidget.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.6.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_IKWidget_t {
    QByteArrayData data[10];
    char stringdata0[105];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_IKWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_IKWidget_t qt_meta_stringdata_IKWidget = {
    {
QT_MOC_LITERAL(0, 0, 8), // "IKWidget"
QT_MOC_LITERAL(1, 9, 7), // "animate"
QT_MOC_LITERAL(2, 17, 0), // ""
QT_MOC_LITERAL(3, 18, 12), // "selectSolver"
QT_MOC_LITERAL(4, 31, 5), // "index"
QT_MOC_LITERAL(5, 37, 17), // "changeJointNumber"
QT_MOC_LITERAL(6, 55, 5), // "value"
QT_MOC_LITERAL(7, 61, 15), // "changeDrawDebug"
QT_MOC_LITERAL(8, 77, 5), // "state"
QT_MOC_LITERAL(9, 83, 21) // "changeDrawComputation"

    },
    "IKWidget\0animate\0\0selectSolver\0index\0"
    "changeJointNumber\0value\0changeDrawDebug\0"
    "state\0changeDrawComputation"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_IKWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x0a /* Public */,
       3,    1,   40,    2, 0x0a /* Public */,
       5,    1,   43,    2, 0x0a /* Public */,
       7,    1,   46,    2, 0x0a /* Public */,
       9,    1,   49,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    4,
    QMetaType::Void, QMetaType::Int,    6,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void, QMetaType::Int,    8,

       0        // eod
};

void IKWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        IKWidget *_t = static_cast<IKWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->animate(); break;
        case 1: _t->selectSolver((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->changeJointNumber((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->changeDrawDebug((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->changeDrawComputation((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject IKWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_IKWidget.data,
      qt_meta_data_IKWidget,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *IKWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *IKWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_IKWidget.stringdata0))
        return static_cast<void*>(const_cast< IKWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int IKWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
