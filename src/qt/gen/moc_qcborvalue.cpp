/****************************************************************************
** Meta object code from reading C++ file 'qcborvalue.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/qbase/src/corelib/serialization/qcborvalue.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qcborvalue.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QCborValue_t {
    QByteArrayData data[19];
    char stringdata0[143];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QCborValue_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QCborValue_t qt_meta_stringdata_QCborValue = {
    {
QT_MOC_LITERAL(0, 0, 10), // "QCborValue"
QT_MOC_LITERAL(1, 11, 4), // "Type"
QT_MOC_LITERAL(2, 16, 7), // "Integer"
QT_MOC_LITERAL(3, 24, 9), // "ByteArray"
QT_MOC_LITERAL(4, 34, 6), // "String"
QT_MOC_LITERAL(5, 41, 5), // "Array"
QT_MOC_LITERAL(6, 47, 3), // "Map"
QT_MOC_LITERAL(7, 51, 3), // "Tag"
QT_MOC_LITERAL(8, 55, 10), // "SimpleType"
QT_MOC_LITERAL(9, 66, 5), // "False"
QT_MOC_LITERAL(10, 72, 4), // "True"
QT_MOC_LITERAL(11, 77, 4), // "Null"
QT_MOC_LITERAL(12, 82, 9), // "Undefined"
QT_MOC_LITERAL(13, 92, 6), // "Double"
QT_MOC_LITERAL(14, 99, 8), // "DateTime"
QT_MOC_LITERAL(15, 108, 3), // "Url"
QT_MOC_LITERAL(16, 112, 17), // "RegularExpression"
QT_MOC_LITERAL(17, 130, 4), // "Uuid"
QT_MOC_LITERAL(18, 135, 7) // "Invalid"

    },
    "QCborValue\0Type\0Integer\0ByteArray\0"
    "String\0Array\0Map\0Tag\0SimpleType\0False\0"
    "True\0Null\0Undefined\0Double\0DateTime\0"
    "Url\0RegularExpression\0Uuid\0Invalid"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QCborValue[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       1,   14, // enums/sets
       0,    0, // constructors
       4,       // flags
       0,       // signalCount

 // enums: name, alias, flags, count, data
       1,    1, 0x0,   17,   19,

 // enum data: key, value
       2, uint(QCborValue::Integer),
       3, uint(QCborValue::ByteArray),
       4, uint(QCborValue::String),
       5, uint(QCborValue::Array),
       6, uint(QCborValue::Map),
       7, uint(QCborValue::Tag),
       8, uint(QCborValue::SimpleType),
       9, uint(QCborValue::False),
      10, uint(QCborValue::True),
      11, uint(QCborValue::Null),
      12, uint(QCborValue::Undefined),
      13, uint(QCborValue::Double),
      14, uint(QCborValue::DateTime),
      15, uint(QCborValue::Url),
      16, uint(QCborValue::RegularExpression),
      17, uint(QCborValue::Uuid),
      18, uint(QCborValue::Invalid),

       0        // eod
};

QT_INIT_METAOBJECT const QMetaObject QCborValue::staticMetaObject = { {
    nullptr,
    qt_meta_stringdata_QCborValue.data,
    qt_meta_data_QCborValue,
    nullptr,
    nullptr,
    nullptr
} };

QT_WARNING_POP
QT_END_MOC_NAMESPACE
