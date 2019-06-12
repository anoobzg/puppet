/****************************************************************************
** Meta object code from reading C++ file 'qiodevice.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/qbase/src/corelib/io/qiodevice.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qiodevice.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QIODevice_t {
    QByteArrayData data[10];
    char stringdata0[118];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QIODevice_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QIODevice_t qt_meta_stringdata_QIODevice = {
    {
QT_MOC_LITERAL(0, 0, 9), // "QIODevice"
QT_MOC_LITERAL(1, 10, 9), // "readyRead"
QT_MOC_LITERAL(2, 20, 0), // ""
QT_MOC_LITERAL(3, 21, 16), // "channelReadyRead"
QT_MOC_LITERAL(4, 38, 7), // "channel"
QT_MOC_LITERAL(5, 46, 12), // "bytesWritten"
QT_MOC_LITERAL(6, 59, 5), // "bytes"
QT_MOC_LITERAL(7, 65, 19), // "channelBytesWritten"
QT_MOC_LITERAL(8, 85, 12), // "aboutToClose"
QT_MOC_LITERAL(9, 98, 19) // "readChannelFinished"

    },
    "QIODevice\0readyRead\0\0channelReadyRead\0"
    "channel\0bytesWritten\0bytes\0"
    "channelBytesWritten\0aboutToClose\0"
    "readChannelFinished"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QIODevice[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   44,    2, 0x06 /* Public */,
       3,    1,   45,    2, 0x06 /* Public */,
       5,    1,   48,    2, 0x06 /* Public */,
       7,    2,   51,    2, 0x06 /* Public */,
       8,    0,   56,    2, 0x06 /* Public */,
       9,    0,   57,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    4,
    QMetaType::Void, QMetaType::LongLong,    6,
    QMetaType::Void, QMetaType::Int, QMetaType::LongLong,    4,    6,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void QIODevice::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QIODevice *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->readyRead(); break;
        case 1: _t->channelReadyRead((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->bytesWritten((*reinterpret_cast< qint64(*)>(_a[1]))); break;
        case 3: _t->channelBytesWritten((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< qint64(*)>(_a[2]))); break;
        case 4: _t->aboutToClose(); break;
        case 5: _t->readChannelFinished(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QIODevice::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QIODevice::readyRead)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (QIODevice::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QIODevice::channelReadyRead)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (QIODevice::*)(qint64 );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QIODevice::bytesWritten)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (QIODevice::*)(int , qint64 );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QIODevice::channelBytesWritten)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (QIODevice::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QIODevice::aboutToClose)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (QIODevice::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QIODevice::readChannelFinished)) {
                *result = 5;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject QIODevice::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_QIODevice.data,
    qt_meta_data_QIODevice,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QIODevice::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QIODevice::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QIODevice.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int QIODevice::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void QIODevice::readyRead()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void QIODevice::channelReadyRead(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void QIODevice::bytesWritten(qint64 _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void QIODevice::channelBytesWritten(int _t1, qint64 _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void QIODevice::aboutToClose()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}

// SIGNAL 5
void QIODevice::readChannelFinished()
{
    QMetaObject::activate(this, &staticMetaObject, 5, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
