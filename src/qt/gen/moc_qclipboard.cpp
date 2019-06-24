/****************************************************************************
** Meta object code from reading C++ file 'qclipboard.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/qbase/src/gui/kernel/qclipboard.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qclipboard.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QClipboard_t {
    QByteArrayData data[8];
    char stringdata0[89];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QClipboard_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QClipboard_t qt_meta_stringdata_QClipboard = {
    {
QT_MOC_LITERAL(0, 0, 10), // "QClipboard"
QT_MOC_LITERAL(1, 11, 7), // "changed"
QT_MOC_LITERAL(2, 19, 0), // ""
QT_MOC_LITERAL(3, 20, 16), // "QClipboard::Mode"
QT_MOC_LITERAL(4, 37, 4), // "mode"
QT_MOC_LITERAL(5, 42, 16), // "selectionChanged"
QT_MOC_LITERAL(6, 59, 17), // "findBufferChanged"
QT_MOC_LITERAL(7, 77, 11) // "dataChanged"

    },
    "QClipboard\0changed\0\0QClipboard::Mode\0"
    "mode\0selectionChanged\0findBufferChanged\0"
    "dataChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QClipboard[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       5,    0,   37,    2, 0x06 /* Public */,
       6,    0,   38,    2, 0x06 /* Public */,
       7,    0,   39,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void QClipboard::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QClipboard *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->changed((*reinterpret_cast< QClipboard::Mode(*)>(_a[1]))); break;
        case 1: _t->selectionChanged(); break;
        case 2: _t->findBufferChanged(); break;
        case 3: _t->dataChanged(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QClipboard::*)(QClipboard::Mode );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QClipboard::changed)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (QClipboard::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QClipboard::selectionChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (QClipboard::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QClipboard::findBufferChanged)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (QClipboard::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QClipboard::dataChanged)) {
                *result = 3;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject QClipboard::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_QClipboard.data,
    qt_meta_data_QClipboard,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QClipboard::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QClipboard::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QClipboard.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int QClipboard::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void QClipboard::changed(QClipboard::Mode _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void QClipboard::selectionChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void QClipboard::findBufferChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void QClipboard::dataChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
