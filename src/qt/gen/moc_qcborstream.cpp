/****************************************************************************
** Meta object code from reading C++ file 'qcborstream.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/qbase/src/corelib/serialization/qcborstream.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qcborstream.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QCborStreamReader_t {
    QByteArrayData data[21];
    char stringdata0[196];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QCborStreamReader_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QCborStreamReader_t qt_meta_stringdata_QCborStreamReader = {
    {
QT_MOC_LITERAL(0, 0, 17), // "QCborStreamReader"
QT_MOC_LITERAL(1, 18, 4), // "Type"
QT_MOC_LITERAL(2, 23, 15), // "UnsignedInteger"
QT_MOC_LITERAL(3, 39, 15), // "NegativeInteger"
QT_MOC_LITERAL(4, 55, 10), // "ByteString"
QT_MOC_LITERAL(5, 66, 9), // "ByteArray"
QT_MOC_LITERAL(6, 76, 10), // "TextString"
QT_MOC_LITERAL(7, 87, 6), // "String"
QT_MOC_LITERAL(8, 94, 5), // "Array"
QT_MOC_LITERAL(9, 100, 3), // "Map"
QT_MOC_LITERAL(10, 104, 3), // "Tag"
QT_MOC_LITERAL(11, 108, 10), // "SimpleType"
QT_MOC_LITERAL(12, 119, 9), // "HalfFloat"
QT_MOC_LITERAL(13, 129, 7), // "Float16"
QT_MOC_LITERAL(14, 137, 5), // "Float"
QT_MOC_LITERAL(15, 143, 6), // "Double"
QT_MOC_LITERAL(16, 150, 7), // "Invalid"
QT_MOC_LITERAL(17, 158, 16), // "StringResultCode"
QT_MOC_LITERAL(18, 175, 11), // "EndOfString"
QT_MOC_LITERAL(19, 187, 2), // "Ok"
QT_MOC_LITERAL(20, 190, 5) // "Error"

    },
    "QCborStreamReader\0Type\0UnsignedInteger\0"
    "NegativeInteger\0ByteString\0ByteArray\0"
    "TextString\0String\0Array\0Map\0Tag\0"
    "SimpleType\0HalfFloat\0Float16\0Float\0"
    "Double\0Invalid\0StringResultCode\0"
    "EndOfString\0Ok\0Error"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QCborStreamReader[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       2,   14, // enums/sets
       0,    0, // constructors
       4,       // flags
       0,       // signalCount

 // enums: name, alias, flags, count, data
       1,    1, 0x0,   15,   24,
      17,   17, 0x0,    3,   54,

 // enum data: key, value
       2, uint(QCborStreamReader::UnsignedInteger),
       3, uint(QCborStreamReader::NegativeInteger),
       4, uint(QCborStreamReader::ByteString),
       5, uint(QCborStreamReader::ByteArray),
       6, uint(QCborStreamReader::TextString),
       7, uint(QCborStreamReader::String),
       8, uint(QCborStreamReader::Array),
       9, uint(QCborStreamReader::Map),
      10, uint(QCborStreamReader::Tag),
      11, uint(QCborStreamReader::SimpleType),
      12, uint(QCborStreamReader::HalfFloat),
      13, uint(QCborStreamReader::Float16),
      14, uint(QCborStreamReader::Float),
      15, uint(QCborStreamReader::Double),
      16, uint(QCborStreamReader::Invalid),
      18, uint(QCborStreamReader::EndOfString),
      19, uint(QCborStreamReader::Ok),
      20, uint(QCborStreamReader::Error),

       0        // eod
};

QT_INIT_METAOBJECT const QMetaObject QCborStreamReader::staticMetaObject = { {
    nullptr,
    qt_meta_stringdata_QCborStreamReader.data,
    qt_meta_data_QCborStreamReader,
    nullptr,
    nullptr,
    nullptr
} };

QT_WARNING_POP
QT_END_MOC_NAMESPACE
