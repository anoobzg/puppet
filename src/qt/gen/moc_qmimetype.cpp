/****************************************************************************
** Meta object code from reading C++ file 'qmimetype.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/qbase/src/corelib/mimetypes/qmimetype.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qmimetype.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QMimeType_t {
    QByteArrayData data[17];
    char stringdata0[175];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QMimeType_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QMimeType_t qt_meta_stringdata_QMimeType = {
    {
QT_MOC_LITERAL(0, 0, 9), // "QMimeType"
QT_MOC_LITERAL(1, 10, 8), // "inherits"
QT_MOC_LITERAL(2, 19, 0), // ""
QT_MOC_LITERAL(3, 20, 12), // "mimeTypeName"
QT_MOC_LITERAL(4, 33, 5), // "valid"
QT_MOC_LITERAL(5, 39, 9), // "isDefault"
QT_MOC_LITERAL(6, 49, 4), // "name"
QT_MOC_LITERAL(7, 54, 7), // "comment"
QT_MOC_LITERAL(8, 62, 15), // "genericIconName"
QT_MOC_LITERAL(9, 78, 8), // "iconName"
QT_MOC_LITERAL(10, 87, 12), // "globPatterns"
QT_MOC_LITERAL(11, 100, 15), // "parentMimeTypes"
QT_MOC_LITERAL(12, 116, 12), // "allAncestors"
QT_MOC_LITERAL(13, 129, 7), // "aliases"
QT_MOC_LITERAL(14, 137, 8), // "suffixes"
QT_MOC_LITERAL(15, 146, 15), // "preferredSuffix"
QT_MOC_LITERAL(16, 162, 12) // "filterString"

    },
    "QMimeType\0inherits\0\0mimeTypeName\0valid\0"
    "isDefault\0name\0comment\0genericIconName\0"
    "iconName\0globPatterns\0parentMimeTypes\0"
    "allAncestors\0aliases\0suffixes\0"
    "preferredSuffix\0filterString"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QMimeType[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
      13,   22, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       4,       // flags
       0,       // signalCount

 // methods: name, argc, parameters, tag, flags
       1,    1,   19,    2, 0x02 /* Public */,

 // methods: parameters
    QMetaType::Bool, QMetaType::QString,    3,

 // properties: name, type, flags
       4, QMetaType::Bool, 0x00095401,
       5, QMetaType::Bool, 0x00095401,
       6, QMetaType::QString, 0x00095401,
       7, QMetaType::QString, 0x00095401,
       8, QMetaType::QString, 0x00095401,
       9, QMetaType::QString, 0x00095401,
      10, QMetaType::QStringList, 0x00095401,
      11, QMetaType::QStringList, 0x00095401,
      12, QMetaType::QStringList, 0x00095401,
      13, QMetaType::QStringList, 0x00095401,
      14, QMetaType::QStringList, 0x00095401,
      15, QMetaType::QString, 0x00095401,
      16, QMetaType::QString, 0x00095401,

       0        // eod
};

void QMimeType::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = reinterpret_cast<QMimeType *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: { bool _r = _t->inherits((*reinterpret_cast< const QString(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty) {
        auto *_t = reinterpret_cast<QMimeType *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< bool*>(_v) = _t->isValid(); break;
        case 1: *reinterpret_cast< bool*>(_v) = _t->isDefault(); break;
        case 2: *reinterpret_cast< QString*>(_v) = _t->name(); break;
        case 3: *reinterpret_cast< QString*>(_v) = _t->comment(); break;
        case 4: *reinterpret_cast< QString*>(_v) = _t->genericIconName(); break;
        case 5: *reinterpret_cast< QString*>(_v) = _t->iconName(); break;
        case 6: *reinterpret_cast< QStringList*>(_v) = _t->globPatterns(); break;
        case 7: *reinterpret_cast< QStringList*>(_v) = _t->parentMimeTypes(); break;
        case 8: *reinterpret_cast< QStringList*>(_v) = _t->allAncestors(); break;
        case 9: *reinterpret_cast< QStringList*>(_v) = _t->aliases(); break;
        case 10: *reinterpret_cast< QStringList*>(_v) = _t->suffixes(); break;
        case 11: *reinterpret_cast< QString*>(_v) = _t->preferredSuffix(); break;
        case 12: *reinterpret_cast< QString*>(_v) = _t->filterString(); break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
    } else if (_c == QMetaObject::ResetProperty) {
    }
#endif // QT_NO_PROPERTIES
}

QT_INIT_METAOBJECT const QMetaObject QMimeType::staticMetaObject = { {
    nullptr,
    qt_meta_stringdata_QMimeType.data,
    qt_meta_data_QMimeType,
    qt_static_metacall,
    nullptr,
    nullptr
} };

QT_WARNING_POP
QT_END_MOC_NAMESPACE
