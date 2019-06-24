/****************************************************************************
** Meta object code from reading C++ file 'qplatformgraphicsbuffer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/qbase/src/gui/kernel/qplatformgraphicsbuffer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qplatformgraphicsbuffer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QPlatformGraphicsBuffer_t {
    QByteArrayData data[14];
    char stringdata0[174];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QPlatformGraphicsBuffer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QPlatformGraphicsBuffer_t qt_meta_stringdata_QPlatformGraphicsBuffer = {
    {
QT_MOC_LITERAL(0, 0, 23), // "QPlatformGraphicsBuffer"
QT_MOC_LITERAL(1, 24, 8), // "unlocked"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 11), // "AccessTypes"
QT_MOC_LITERAL(4, 46, 19), // "previousAccessTypes"
QT_MOC_LITERAL(5, 66, 10), // "AccessType"
QT_MOC_LITERAL(6, 77, 4), // "None"
QT_MOC_LITERAL(7, 82, 12), // "SWReadAccess"
QT_MOC_LITERAL(8, 95, 13), // "SWWriteAccess"
QT_MOC_LITERAL(9, 109, 13), // "TextureAccess"
QT_MOC_LITERAL(10, 123, 12), // "HWCompositor"
QT_MOC_LITERAL(11, 136, 6), // "Origin"
QT_MOC_LITERAL(12, 143, 16), // "OriginBottomLeft"
QT_MOC_LITERAL(13, 160, 13) // "OriginTopLeft"

    },
    "QPlatformGraphicsBuffer\0unlocked\0\0"
    "AccessTypes\0previousAccessTypes\0"
    "AccessType\0None\0SWReadAccess\0SWWriteAccess\0"
    "TextureAccess\0HWCompositor\0Origin\0"
    "OriginBottomLeft\0OriginTopLeft"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QPlatformGraphicsBuffer[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       2,   22, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   19,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

 // enums: name, alias, flags, count, data
       5,    5, 0x0,    5,   32,
      11,   11, 0x0,    2,   42,

 // enum data: key, value
       6, uint(QPlatformGraphicsBuffer::None),
       7, uint(QPlatformGraphicsBuffer::SWReadAccess),
       8, uint(QPlatformGraphicsBuffer::SWWriteAccess),
       9, uint(QPlatformGraphicsBuffer::TextureAccess),
      10, uint(QPlatformGraphicsBuffer::HWCompositor),
      12, uint(QPlatformGraphicsBuffer::OriginBottomLeft),
      13, uint(QPlatformGraphicsBuffer::OriginTopLeft),

       0        // eod
};

void QPlatformGraphicsBuffer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QPlatformGraphicsBuffer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->unlocked((*reinterpret_cast< AccessTypes(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QPlatformGraphicsBuffer::*)(AccessTypes );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QPlatformGraphicsBuffer::unlocked)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject QPlatformGraphicsBuffer::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_QPlatformGraphicsBuffer.data,
    qt_meta_data_QPlatformGraphicsBuffer,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QPlatformGraphicsBuffer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QPlatformGraphicsBuffer::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QPlatformGraphicsBuffer.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int QPlatformGraphicsBuffer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void QPlatformGraphicsBuffer::unlocked(AccessTypes _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
