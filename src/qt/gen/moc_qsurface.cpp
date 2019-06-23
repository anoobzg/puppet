/****************************************************************************
** Meta object code from reading C++ file 'qsurface.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/qbase/src/gui/kernel/qsurface.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qsurface.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QSurface_t {
    QByteArrayData data[11];
    char stringdata0[136];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QSurface_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QSurface_t qt_meta_stringdata_QSurface = {
    {
QT_MOC_LITERAL(0, 0, 8), // "QSurface"
QT_MOC_LITERAL(1, 9, 12), // "SurfaceClass"
QT_MOC_LITERAL(2, 22, 6), // "Window"
QT_MOC_LITERAL(3, 29, 9), // "Offscreen"
QT_MOC_LITERAL(4, 39, 11), // "SurfaceType"
QT_MOC_LITERAL(5, 51, 13), // "RasterSurface"
QT_MOC_LITERAL(6, 65, 13), // "OpenGLSurface"
QT_MOC_LITERAL(7, 79, 15), // "RasterGLSurface"
QT_MOC_LITERAL(8, 95, 13), // "OpenVGSurface"
QT_MOC_LITERAL(9, 109, 13), // "VulkanSurface"
QT_MOC_LITERAL(10, 123, 12) // "MetalSurface"

    },
    "QSurface\0SurfaceClass\0Window\0Offscreen\0"
    "SurfaceType\0RasterSurface\0OpenGLSurface\0"
    "RasterGLSurface\0OpenVGSurface\0"
    "VulkanSurface\0MetalSurface"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QSurface[] = {

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
       1,    1, 0x0,    2,   24,
       4,    4, 0x0,    6,   28,

 // enum data: key, value
       2, uint(QSurface::Window),
       3, uint(QSurface::Offscreen),
       5, uint(QSurface::RasterSurface),
       6, uint(QSurface::OpenGLSurface),
       7, uint(QSurface::RasterGLSurface),
       8, uint(QSurface::OpenVGSurface),
       9, uint(QSurface::VulkanSurface),
      10, uint(QSurface::MetalSurface),

       0        // eod
};

QT_INIT_METAOBJECT const QMetaObject QSurface::staticMetaObject = { {
    nullptr,
    qt_meta_stringdata_QSurface.data,
    qt_meta_data_QSurface,
    nullptr,
    nullptr,
    nullptr
} };

QT_WARNING_POP
QT_END_MOC_NAMESPACE
