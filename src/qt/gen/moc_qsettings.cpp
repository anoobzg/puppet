/****************************************************************************
** Meta object code from reading C++ file 'qsettings.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/qbase/src/corelib/io/qsettings.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qsettings.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QSettings_t {
    QByteArrayData data[28];
    char stringdata0[352];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QSettings_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QSettings_t qt_meta_stringdata_QSettings = {
    {
QT_MOC_LITERAL(0, 0, 9), // "QSettings"
QT_MOC_LITERAL(1, 10, 6), // "Status"
QT_MOC_LITERAL(2, 17, 7), // "NoError"
QT_MOC_LITERAL(3, 25, 11), // "AccessError"
QT_MOC_LITERAL(4, 37, 11), // "FormatError"
QT_MOC_LITERAL(5, 49, 6), // "Format"
QT_MOC_LITERAL(6, 56, 12), // "NativeFormat"
QT_MOC_LITERAL(7, 69, 9), // "IniFormat"
QT_MOC_LITERAL(8, 79, 13), // "InvalidFormat"
QT_MOC_LITERAL(9, 93, 13), // "CustomFormat1"
QT_MOC_LITERAL(10, 107, 13), // "CustomFormat2"
QT_MOC_LITERAL(11, 121, 13), // "CustomFormat3"
QT_MOC_LITERAL(12, 135, 13), // "CustomFormat4"
QT_MOC_LITERAL(13, 149, 13), // "CustomFormat5"
QT_MOC_LITERAL(14, 163, 13), // "CustomFormat6"
QT_MOC_LITERAL(15, 177, 13), // "CustomFormat7"
QT_MOC_LITERAL(16, 191, 13), // "CustomFormat8"
QT_MOC_LITERAL(17, 205, 13), // "CustomFormat9"
QT_MOC_LITERAL(18, 219, 14), // "CustomFormat10"
QT_MOC_LITERAL(19, 234, 14), // "CustomFormat11"
QT_MOC_LITERAL(20, 249, 14), // "CustomFormat12"
QT_MOC_LITERAL(21, 264, 14), // "CustomFormat13"
QT_MOC_LITERAL(22, 279, 14), // "CustomFormat14"
QT_MOC_LITERAL(23, 294, 14), // "CustomFormat15"
QT_MOC_LITERAL(24, 309, 14), // "CustomFormat16"
QT_MOC_LITERAL(25, 324, 5), // "Scope"
QT_MOC_LITERAL(26, 330, 9), // "UserScope"
QT_MOC_LITERAL(27, 340, 11) // "SystemScope"

    },
    "QSettings\0Status\0NoError\0AccessError\0"
    "FormatError\0Format\0NativeFormat\0"
    "IniFormat\0InvalidFormat\0CustomFormat1\0"
    "CustomFormat2\0CustomFormat3\0CustomFormat4\0"
    "CustomFormat5\0CustomFormat6\0CustomFormat7\0"
    "CustomFormat8\0CustomFormat9\0CustomFormat10\0"
    "CustomFormat11\0CustomFormat12\0"
    "CustomFormat13\0CustomFormat14\0"
    "CustomFormat15\0CustomFormat16\0Scope\0"
    "UserScope\0SystemScope"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QSettings[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       3,   14, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // enums: name, alias, flags, count, data
       1,    1, 0x0,    3,   29,
       5,    5, 0x0,   19,   35,
      25,   25, 0x0,    2,   73,

 // enum data: key, value
       2, uint(QSettings::NoError),
       3, uint(QSettings::AccessError),
       4, uint(QSettings::FormatError),
       6, uint(QSettings::NativeFormat),
       7, uint(QSettings::IniFormat),
       8, uint(QSettings::InvalidFormat),
       9, uint(QSettings::CustomFormat1),
      10, uint(QSettings::CustomFormat2),
      11, uint(QSettings::CustomFormat3),
      12, uint(QSettings::CustomFormat4),
      13, uint(QSettings::CustomFormat5),
      14, uint(QSettings::CustomFormat6),
      15, uint(QSettings::CustomFormat7),
      16, uint(QSettings::CustomFormat8),
      17, uint(QSettings::CustomFormat9),
      18, uint(QSettings::CustomFormat10),
      19, uint(QSettings::CustomFormat11),
      20, uint(QSettings::CustomFormat12),
      21, uint(QSettings::CustomFormat13),
      22, uint(QSettings::CustomFormat14),
      23, uint(QSettings::CustomFormat15),
      24, uint(QSettings::CustomFormat16),
      26, uint(QSettings::UserScope),
      27, uint(QSettings::SystemScope),

       0        // eod
};

void QSettings::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject QSettings::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_QSettings.data,
    qt_meta_data_QSettings,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QSettings::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QSettings::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QSettings.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int QSettings::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
