/****************************************************************************
** Meta object code from reading C++ file 'qabstractproxymodel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/qbase/src/corelib/itemmodels/qabstractproxymodel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qabstractproxymodel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QAbstractProxyModel_t {
    QByteArrayData data[16];
    char stringdata0[244];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QAbstractProxyModel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QAbstractProxyModel_t qt_meta_stringdata_QAbstractProxyModel = {
    {
QT_MOC_LITERAL(0, 0, 19), // "QAbstractProxyModel"
QT_MOC_LITERAL(1, 20, 18), // "sourceModelChanged"
QT_MOC_LITERAL(2, 39, 0), // ""
QT_MOC_LITERAL(3, 40, 17), // "resetInternalData"
QT_MOC_LITERAL(4, 58, 23), // "_q_sourceModelDestroyed"
QT_MOC_LITERAL(5, 82, 11), // "mapToSource"
QT_MOC_LITERAL(6, 94, 11), // "QModelIndex"
QT_MOC_LITERAL(7, 106, 10), // "proxyIndex"
QT_MOC_LITERAL(8, 117, 13), // "mapFromSource"
QT_MOC_LITERAL(9, 131, 11), // "sourceIndex"
QT_MOC_LITERAL(10, 143, 20), // "mapSelectionToSource"
QT_MOC_LITERAL(11, 164, 14), // "QItemSelection"
QT_MOC_LITERAL(12, 179, 9), // "selection"
QT_MOC_LITERAL(13, 189, 22), // "mapSelectionFromSource"
QT_MOC_LITERAL(14, 212, 11), // "sourceModel"
QT_MOC_LITERAL(15, 224, 19) // "QAbstractItemModel*"

    },
    "QAbstractProxyModel\0sourceModelChanged\0"
    "\0resetInternalData\0_q_sourceModelDestroyed\0"
    "mapToSource\0QModelIndex\0proxyIndex\0"
    "mapFromSource\0sourceIndex\0"
    "mapSelectionToSource\0QItemSelection\0"
    "selection\0mapSelectionFromSource\0"
    "sourceModel\0QAbstractItemModel*"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QAbstractProxyModel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       1,   64, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   49,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   50,    2, 0x09 /* Protected */,
       4,    0,   51,    2, 0x08 /* Private */,

 // methods: name, argc, parameters, tag, flags
       5,    1,   52,    2, 0x02 /* Public */,
       8,    1,   55,    2, 0x02 /* Public */,
      10,    1,   58,    2, 0x02 /* Public */,
      13,    1,   61,    2, 0x02 /* Public */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

 // methods: parameters
    0x80000000 | 6, 0x80000000 | 6,    7,
    0x80000000 | 6, 0x80000000 | 6,    9,
    0x80000000 | 11, 0x80000000 | 11,   12,
    0x80000000 | 11, 0x80000000 | 11,   12,

 // properties: name, type, flags
      14, 0x80000000 | 15, 0x0049510b,

 // properties: notify_signal_id
       0,

       0        // eod
};

void QAbstractProxyModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QAbstractProxyModel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->sourceModelChanged(QPrivateSignal()); break;
        case 1: _t->resetInternalData(); break;
        case 2: _t->d_func()->_q_sourceModelDestroyed(); break;
        case 3: { QModelIndex _r = _t->mapToSource((*reinterpret_cast< const QModelIndex(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< QModelIndex*>(_a[0]) = std::move(_r); }  break;
        case 4: { QModelIndex _r = _t->mapFromSource((*reinterpret_cast< const QModelIndex(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< QModelIndex*>(_a[0]) = std::move(_r); }  break;
        case 5: { QItemSelection _r = _t->mapSelectionToSource((*reinterpret_cast< const QItemSelection(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< QItemSelection*>(_a[0]) = std::move(_r); }  break;
        case 6: { QItemSelection _r = _t->mapSelectionFromSource((*reinterpret_cast< const QItemSelection(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< QItemSelection*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QAbstractProxyModel::*)(QPrivateSignal);
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractProxyModel::sourceModelChanged)) {
                *result = 0;
                return;
            }
        }
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty) {
        auto *_t = static_cast<QAbstractProxyModel *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< QAbstractItemModel**>(_v) = _t->sourceModel(); break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        auto *_t = static_cast<QAbstractProxyModel *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: _t->setSourceModel(*reinterpret_cast< QAbstractItemModel**>(_v)); break;
        default: break;
        }
    } else if (_c == QMetaObject::ResetProperty) {
    }
#endif // QT_NO_PROPERTIES
}

QT_INIT_METAOBJECT const QMetaObject QAbstractProxyModel::staticMetaObject = { {
    &QAbstractItemModel::staticMetaObject,
    qt_meta_stringdata_QAbstractProxyModel.data,
    qt_meta_data_QAbstractProxyModel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QAbstractProxyModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QAbstractProxyModel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QAbstractProxyModel.stringdata0))
        return static_cast<void*>(this);
    return QAbstractItemModel::qt_metacast(_clname);
}

int QAbstractProxyModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QAbstractItemModel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
#ifndef QT_NO_PROPERTIES
   else if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 1;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 1;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 1;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 1;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 1;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}

// SIGNAL 0
void QAbstractProxyModel::sourceModelChanged(QPrivateSignal _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
