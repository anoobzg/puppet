/****************************************************************************
** Meta object code from reading C++ file 'qcborcommon.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/qbase/src/corelib/serialization/qcborcommon.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qcborcommon.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QCborError_t {
    QByteArrayData data[17];
    char stringdata0[226];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QCborError_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QCborError_t qt_meta_stringdata_QCborError = {
    {
QT_MOC_LITERAL(0, 0, 10), // "QCborError"
QT_MOC_LITERAL(1, 11, 4), // "Code"
QT_MOC_LITERAL(2, 16, 12), // "UnknownError"
QT_MOC_LITERAL(3, 29, 14), // "AdvancePastEnd"
QT_MOC_LITERAL(4, 44, 16), // "InputOutputError"
QT_MOC_LITERAL(5, 61, 12), // "GarbageAtEnd"
QT_MOC_LITERAL(6, 74, 9), // "EndOfFile"
QT_MOC_LITERAL(7, 84, 15), // "UnexpectedBreak"
QT_MOC_LITERAL(8, 100, 11), // "UnknownType"
QT_MOC_LITERAL(9, 112, 11), // "IllegalType"
QT_MOC_LITERAL(10, 124, 13), // "IllegalNumber"
QT_MOC_LITERAL(11, 138, 17), // "IllegalSimpleType"
QT_MOC_LITERAL(12, 156, 17), // "InvalidUtf8String"
QT_MOC_LITERAL(13, 174, 12), // "DataTooLarge"
QT_MOC_LITERAL(14, 187, 14), // "NestingTooDeep"
QT_MOC_LITERAL(15, 202, 15), // "UnsupportedType"
QT_MOC_LITERAL(16, 218, 7) // "NoError"

    },
    "QCborError\0Code\0UnknownError\0"
    "AdvancePastEnd\0InputOutputError\0"
    "GarbageAtEnd\0EndOfFile\0UnexpectedBreak\0"
    "UnknownType\0IllegalType\0IllegalNumber\0"
    "IllegalSimpleType\0InvalidUtf8String\0"
    "DataTooLarge\0NestingTooDeep\0UnsupportedType\0"
    "NoError"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QCborError[] = {

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
       1,    1, 0x0,   15,   19,

 // enum data: key, value
       2, uint(QCborError::UnknownError),
       3, uint(QCborError::AdvancePastEnd),
       4, uint(QCborError::InputOutputError),
       5, uint(QCborError::GarbageAtEnd),
       6, uint(QCborError::EndOfFile),
       7, uint(QCborError::UnexpectedBreak),
       8, uint(QCborError::UnknownType),
       9, uint(QCborError::IllegalType),
      10, uint(QCborError::IllegalNumber),
      11, uint(QCborError::IllegalSimpleType),
      12, uint(QCborError::InvalidUtf8String),
      13, uint(QCborError::DataTooLarge),
      14, uint(QCborError::NestingTooDeep),
      15, uint(QCborError::UnsupportedType),
      16, uint(QCborError::NoError),

       0        // eod
};

QT_INIT_METAOBJECT const QMetaObject QCborError::staticMetaObject = { {
    nullptr,
    qt_meta_stringdata_QCborError.data,
    qt_meta_data_QCborError,
    nullptr,
    nullptr,
    nullptr
} };

QT_WARNING_POP
QT_END_MOC_NAMESPACE
