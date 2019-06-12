/****************************************************************************
** Meta object code from reading C++ file 'qabstractitemmodel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/qbase/src/corelib/itemmodels/qabstractitemmodel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>
#include <QtCore/QVector>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qabstractitemmodel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QAbstractItemModel_t {
    QByteArrayData data[79];
    char stringdata0[993];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QAbstractItemModel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QAbstractItemModel_t qt_meta_stringdata_QAbstractItemModel = {
    {
QT_MOC_LITERAL(0, 0, 18), // "QAbstractItemModel"
QT_MOC_LITERAL(1, 19, 11), // "dataChanged"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 11), // "QModelIndex"
QT_MOC_LITERAL(4, 44, 7), // "topLeft"
QT_MOC_LITERAL(5, 52, 11), // "bottomRight"
QT_MOC_LITERAL(6, 64, 12), // "QVector<int>"
QT_MOC_LITERAL(7, 77, 5), // "roles"
QT_MOC_LITERAL(8, 83, 17), // "headerDataChanged"
QT_MOC_LITERAL(9, 101, 15), // "Qt::Orientation"
QT_MOC_LITERAL(10, 117, 11), // "orientation"
QT_MOC_LITERAL(11, 129, 5), // "first"
QT_MOC_LITERAL(12, 135, 4), // "last"
QT_MOC_LITERAL(13, 140, 13), // "layoutChanged"
QT_MOC_LITERAL(14, 154, 28), // "QList<QPersistentModelIndex>"
QT_MOC_LITERAL(15, 183, 7), // "parents"
QT_MOC_LITERAL(16, 191, 36), // "QAbstractItemModel::LayoutCha..."
QT_MOC_LITERAL(17, 228, 4), // "hint"
QT_MOC_LITERAL(18, 233, 22), // "layoutAboutToBeChanged"
QT_MOC_LITERAL(19, 256, 21), // "rowsAboutToBeInserted"
QT_MOC_LITERAL(20, 278, 6), // "parent"
QT_MOC_LITERAL(21, 285, 12), // "rowsInserted"
QT_MOC_LITERAL(22, 298, 20), // "rowsAboutToBeRemoved"
QT_MOC_LITERAL(23, 319, 11), // "rowsRemoved"
QT_MOC_LITERAL(24, 331, 24), // "columnsAboutToBeInserted"
QT_MOC_LITERAL(25, 356, 15), // "columnsInserted"
QT_MOC_LITERAL(26, 372, 23), // "columnsAboutToBeRemoved"
QT_MOC_LITERAL(27, 396, 14), // "columnsRemoved"
QT_MOC_LITERAL(28, 411, 19), // "modelAboutToBeReset"
QT_MOC_LITERAL(29, 431, 10), // "modelReset"
QT_MOC_LITERAL(30, 442, 18), // "rowsAboutToBeMoved"
QT_MOC_LITERAL(31, 461, 12), // "sourceParent"
QT_MOC_LITERAL(32, 474, 11), // "sourceStart"
QT_MOC_LITERAL(33, 486, 9), // "sourceEnd"
QT_MOC_LITERAL(34, 496, 17), // "destinationParent"
QT_MOC_LITERAL(35, 514, 14), // "destinationRow"
QT_MOC_LITERAL(36, 529, 9), // "rowsMoved"
QT_MOC_LITERAL(37, 539, 5), // "start"
QT_MOC_LITERAL(38, 545, 3), // "end"
QT_MOC_LITERAL(39, 549, 11), // "destination"
QT_MOC_LITERAL(40, 561, 3), // "row"
QT_MOC_LITERAL(41, 565, 21), // "columnsAboutToBeMoved"
QT_MOC_LITERAL(42, 587, 17), // "destinationColumn"
QT_MOC_LITERAL(43, 605, 12), // "columnsMoved"
QT_MOC_LITERAL(44, 618, 6), // "column"
QT_MOC_LITERAL(45, 625, 6), // "submit"
QT_MOC_LITERAL(46, 632, 6), // "revert"
QT_MOC_LITERAL(47, 639, 17), // "resetInternalData"
QT_MOC_LITERAL(48, 657, 8), // "hasIndex"
QT_MOC_LITERAL(49, 666, 5), // "index"
QT_MOC_LITERAL(50, 672, 5), // "child"
QT_MOC_LITERAL(51, 678, 7), // "sibling"
QT_MOC_LITERAL(52, 686, 3), // "idx"
QT_MOC_LITERAL(53, 690, 8), // "rowCount"
QT_MOC_LITERAL(54, 699, 11), // "columnCount"
QT_MOC_LITERAL(55, 711, 11), // "hasChildren"
QT_MOC_LITERAL(56, 723, 4), // "data"
QT_MOC_LITERAL(57, 728, 4), // "role"
QT_MOC_LITERAL(58, 733, 7), // "setData"
QT_MOC_LITERAL(59, 741, 5), // "value"
QT_MOC_LITERAL(60, 747, 10), // "headerData"
QT_MOC_LITERAL(61, 758, 7), // "section"
QT_MOC_LITERAL(62, 766, 9), // "fetchMore"
QT_MOC_LITERAL(63, 776, 12), // "canFetchMore"
QT_MOC_LITERAL(64, 789, 5), // "flags"
QT_MOC_LITERAL(65, 795, 13), // "Qt::ItemFlags"
QT_MOC_LITERAL(66, 809, 5), // "match"
QT_MOC_LITERAL(67, 815, 15), // "QModelIndexList"
QT_MOC_LITERAL(68, 831, 4), // "hits"
QT_MOC_LITERAL(69, 836, 14), // "Qt::MatchFlags"
QT_MOC_LITERAL(70, 851, 16), // "LayoutChangeHint"
QT_MOC_LITERAL(71, 868, 18), // "NoLayoutChangeHint"
QT_MOC_LITERAL(72, 887, 16), // "VerticalSortHint"
QT_MOC_LITERAL(73, 904, 18), // "HorizontalSortHint"
QT_MOC_LITERAL(74, 923, 16), // "CheckIndexOption"
QT_MOC_LITERAL(75, 940, 8), // "NoOption"
QT_MOC_LITERAL(76, 949, 12), // "IndexIsValid"
QT_MOC_LITERAL(77, 962, 14), // "DoNotUseParent"
QT_MOC_LITERAL(78, 977, 15) // "ParentIsInvalid"

    },
    "QAbstractItemModel\0dataChanged\0\0"
    "QModelIndex\0topLeft\0bottomRight\0"
    "QVector<int>\0roles\0headerDataChanged\0"
    "Qt::Orientation\0orientation\0first\0"
    "last\0layoutChanged\0QList<QPersistentModelIndex>\0"
    "parents\0QAbstractItemModel::LayoutChangeHint\0"
    "hint\0layoutAboutToBeChanged\0"
    "rowsAboutToBeInserted\0parent\0rowsInserted\0"
    "rowsAboutToBeRemoved\0rowsRemoved\0"
    "columnsAboutToBeInserted\0columnsInserted\0"
    "columnsAboutToBeRemoved\0columnsRemoved\0"
    "modelAboutToBeReset\0modelReset\0"
    "rowsAboutToBeMoved\0sourceParent\0"
    "sourceStart\0sourceEnd\0destinationParent\0"
    "destinationRow\0rowsMoved\0start\0end\0"
    "destination\0row\0columnsAboutToBeMoved\0"
    "destinationColumn\0columnsMoved\0column\0"
    "submit\0revert\0resetInternalData\0"
    "hasIndex\0index\0child\0sibling\0idx\0"
    "rowCount\0columnCount\0hasChildren\0data\0"
    "role\0setData\0value\0headerData\0section\0"
    "fetchMore\0canFetchMore\0flags\0Qt::ItemFlags\0"
    "match\0QModelIndexList\0hits\0Qt::MatchFlags\0"
    "LayoutChangeHint\0NoLayoutChangeHint\0"
    "VerticalSortHint\0HorizontalSortHint\0"
    "CheckIndexOption\0NoOption\0IndexIsValid\0"
    "DoNotUseParent\0ParentIsInvalid"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QAbstractItemModel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      50,   14, // methods
       0,    0, // properties
       2,  520, // enums/sets
       0,    0, // constructors
       0,       // flags
      23,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    3,  264,    2, 0x06 /* Public */,
       1,    2,  271,    2, 0x26 /* Public | MethodCloned */,
       8,    3,  276,    2, 0x06 /* Public */,
      13,    2,  283,    2, 0x06 /* Public */,
      13,    1,  288,    2, 0x26 /* Public | MethodCloned */,
      13,    0,  291,    2, 0x26 /* Public | MethodCloned */,
      18,    2,  292,    2, 0x06 /* Public */,
      18,    1,  297,    2, 0x26 /* Public | MethodCloned */,
      18,    0,  300,    2, 0x26 /* Public | MethodCloned */,
      19,    3,  301,    2, 0x06 /* Public */,
      21,    3,  308,    2, 0x06 /* Public */,
      22,    3,  315,    2, 0x06 /* Public */,
      23,    3,  322,    2, 0x06 /* Public */,
      24,    3,  329,    2, 0x06 /* Public */,
      25,    3,  336,    2, 0x06 /* Public */,
      26,    3,  343,    2, 0x06 /* Public */,
      27,    3,  350,    2, 0x06 /* Public */,
      28,    0,  357,    2, 0x06 /* Public */,
      29,    0,  358,    2, 0x06 /* Public */,
      30,    5,  359,    2, 0x06 /* Public */,
      36,    5,  370,    2, 0x06 /* Public */,
      41,    5,  381,    2, 0x06 /* Public */,
      43,    5,  392,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      45,    0,  403,    2, 0x0a /* Public */,
      46,    0,  404,    2, 0x0a /* Public */,
      47,    0,  405,    2, 0x09 /* Protected */,

 // methods: name, argc, parameters, tag, flags
      48,    3,  406,    2, 0x02 /* Public */,
      48,    2,  413,    2, 0x22 /* Public | MethodCloned */,
      49,    3,  418,    2, 0x02 /* Public */,
      49,    2,  425,    2, 0x22 /* Public | MethodCloned */,
      20,    1,  430,    2, 0x02 /* Public */,
      51,    3,  433,    2, 0x02 /* Public */,
      53,    1,  440,    2, 0x02 /* Public */,
      53,    0,  443,    2, 0x22 /* Public | MethodCloned */,
      54,    1,  444,    2, 0x02 /* Public */,
      54,    0,  447,    2, 0x22 /* Public | MethodCloned */,
      55,    1,  448,    2, 0x02 /* Public */,
      55,    0,  451,    2, 0x22 /* Public | MethodCloned */,
      56,    2,  452,    2, 0x02 /* Public */,
      56,    1,  457,    2, 0x22 /* Public | MethodCloned */,
      58,    3,  460,    2, 0x02 /* Public */,
      58,    2,  467,    2, 0x22 /* Public | MethodCloned */,
      60,    3,  472,    2, 0x02 /* Public */,
      60,    2,  479,    2, 0x22 /* Public | MethodCloned */,
      62,    1,  484,    2, 0x02 /* Public */,
      63,    1,  487,    2, 0x02 /* Public */,
      64,    1,  490,    2, 0x02 /* Public */,
      66,    5,  493,    2, 0x02 /* Public */,
      66,    4,  504,    2, 0x22 /* Public | MethodCloned */,
      66,    3,  513,    2, 0x22 /* Public | MethodCloned */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 3, 0x80000000 | 6,    4,    5,    7,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 3,    4,    5,
    QMetaType::Void, 0x80000000 | 9, QMetaType::Int, QMetaType::Int,   10,   11,   12,
    QMetaType::Void, 0x80000000 | 14, 0x80000000 | 16,   15,   17,
    QMetaType::Void, 0x80000000 | 14,   15,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 14, 0x80000000 | 16,   15,   17,
    QMetaType::Void, 0x80000000 | 14,   15,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int, QMetaType::Int,   20,   11,   12,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int, QMetaType::Int,   20,   11,   12,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int, QMetaType::Int,   20,   11,   12,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int, QMetaType::Int,   20,   11,   12,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int, QMetaType::Int,   20,   11,   12,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int, QMetaType::Int,   20,   11,   12,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int, QMetaType::Int,   20,   11,   12,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int, QMetaType::Int,   20,   11,   12,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int, QMetaType::Int, 0x80000000 | 3, QMetaType::Int,   31,   32,   33,   34,   35,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int, QMetaType::Int, 0x80000000 | 3, QMetaType::Int,   20,   37,   38,   39,   40,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int, QMetaType::Int, 0x80000000 | 3, QMetaType::Int,   31,   32,   33,   34,   42,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int, QMetaType::Int, 0x80000000 | 3, QMetaType::Int,   20,   37,   38,   39,   44,

 // slots: parameters
    QMetaType::Bool,
    QMetaType::Void,
    QMetaType::Void,

 // methods: parameters
    QMetaType::Bool, QMetaType::Int, QMetaType::Int, 0x80000000 | 3,   40,   44,   20,
    QMetaType::Bool, QMetaType::Int, QMetaType::Int,   40,   44,
    0x80000000 | 3, QMetaType::Int, QMetaType::Int, 0x80000000 | 3,   40,   44,   20,
    0x80000000 | 3, QMetaType::Int, QMetaType::Int,   40,   44,
    0x80000000 | 3, 0x80000000 | 3,   50,
    0x80000000 | 3, QMetaType::Int, QMetaType::Int, 0x80000000 | 3,   40,   44,   52,
    QMetaType::Int, 0x80000000 | 3,   20,
    QMetaType::Int,
    QMetaType::Int, 0x80000000 | 3,   20,
    QMetaType::Int,
    QMetaType::Bool, 0x80000000 | 3,   20,
    QMetaType::Bool,
    QMetaType::QVariant, 0x80000000 | 3, QMetaType::Int,   49,   57,
    QMetaType::QVariant, 0x80000000 | 3,   49,
    QMetaType::Bool, 0x80000000 | 3, QMetaType::QVariant, QMetaType::Int,   49,   59,   57,
    QMetaType::Bool, 0x80000000 | 3, QMetaType::QVariant,   49,   59,
    QMetaType::QVariant, QMetaType::Int, 0x80000000 | 9, QMetaType::Int,   61,   10,   57,
    QMetaType::QVariant, QMetaType::Int, 0x80000000 | 9,   61,   10,
    QMetaType::Void, 0x80000000 | 3,   20,
    QMetaType::Bool, 0x80000000 | 3,   20,
    0x80000000 | 65, 0x80000000 | 3,   49,
    0x80000000 | 67, 0x80000000 | 3, QMetaType::Int, QMetaType::QVariant, QMetaType::Int, 0x80000000 | 69,   37,   57,   59,   68,   64,
    0x80000000 | 67, 0x80000000 | 3, QMetaType::Int, QMetaType::QVariant, QMetaType::Int,   37,   57,   59,   68,
    0x80000000 | 67, 0x80000000 | 3, QMetaType::Int, QMetaType::QVariant,   37,   57,   59,

 // enums: name, alias, flags, count, data
      70,   70, 0x0,    3,  530,
      74,   74, 0x2,    4,  536,

 // enum data: key, value
      71, uint(QAbstractItemModel::NoLayoutChangeHint),
      72, uint(QAbstractItemModel::VerticalSortHint),
      73, uint(QAbstractItemModel::HorizontalSortHint),
      75, uint(QAbstractItemModel::CheckIndexOption::NoOption),
      76, uint(QAbstractItemModel::CheckIndexOption::IndexIsValid),
      77, uint(QAbstractItemModel::CheckIndexOption::DoNotUseParent),
      78, uint(QAbstractItemModel::CheckIndexOption::ParentIsInvalid),

       0        // eod
};

void QAbstractItemModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QAbstractItemModel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->dataChanged((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< const QModelIndex(*)>(_a[2])),(*reinterpret_cast< const QVector<int>(*)>(_a[3]))); break;
        case 1: _t->dataChanged((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< const QModelIndex(*)>(_a[2]))); break;
        case 2: _t->headerDataChanged((*reinterpret_cast< Qt::Orientation(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 3: _t->layoutChanged((*reinterpret_cast< const QList<QPersistentModelIndex>(*)>(_a[1])),(*reinterpret_cast< QAbstractItemModel::LayoutChangeHint(*)>(_a[2]))); break;
        case 4: _t->layoutChanged((*reinterpret_cast< const QList<QPersistentModelIndex>(*)>(_a[1]))); break;
        case 5: _t->layoutChanged(); break;
        case 6: _t->layoutAboutToBeChanged((*reinterpret_cast< const QList<QPersistentModelIndex>(*)>(_a[1])),(*reinterpret_cast< QAbstractItemModel::LayoutChangeHint(*)>(_a[2]))); break;
        case 7: _t->layoutAboutToBeChanged((*reinterpret_cast< const QList<QPersistentModelIndex>(*)>(_a[1]))); break;
        case 8: _t->layoutAboutToBeChanged(); break;
        case 9: _t->rowsAboutToBeInserted((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])), QPrivateSignal()); break;
        case 10: _t->rowsInserted((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])), QPrivateSignal()); break;
        case 11: _t->rowsAboutToBeRemoved((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])), QPrivateSignal()); break;
        case 12: _t->rowsRemoved((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])), QPrivateSignal()); break;
        case 13: _t->columnsAboutToBeInserted((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])), QPrivateSignal()); break;
        case 14: _t->columnsInserted((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])), QPrivateSignal()); break;
        case 15: _t->columnsAboutToBeRemoved((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])), QPrivateSignal()); break;
        case 16: _t->columnsRemoved((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])), QPrivateSignal()); break;
        case 17: _t->modelAboutToBeReset(QPrivateSignal()); break;
        case 18: _t->modelReset(QPrivateSignal()); break;
        case 19: _t->rowsAboutToBeMoved((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< const QModelIndex(*)>(_a[4])),(*reinterpret_cast< int(*)>(_a[5])), QPrivateSignal()); break;
        case 20: _t->rowsMoved((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< const QModelIndex(*)>(_a[4])),(*reinterpret_cast< int(*)>(_a[5])), QPrivateSignal()); break;
        case 21: _t->columnsAboutToBeMoved((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< const QModelIndex(*)>(_a[4])),(*reinterpret_cast< int(*)>(_a[5])), QPrivateSignal()); break;
        case 22: _t->columnsMoved((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< const QModelIndex(*)>(_a[4])),(*reinterpret_cast< int(*)>(_a[5])), QPrivateSignal()); break;
        case 23: { bool _r = _t->submit();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 24: _t->revert(); break;
        case 25: _t->resetInternalData(); break;
        case 26: { bool _r = _t->hasIndex((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< const QModelIndex(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 27: { bool _r = _t->hasIndex((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 28: { QModelIndex _r = _t->index((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< const QModelIndex(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< QModelIndex*>(_a[0]) = std::move(_r); }  break;
        case 29: { QModelIndex _r = _t->index((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< QModelIndex*>(_a[0]) = std::move(_r); }  break;
        case 30: { QModelIndex _r = _t->parent((*reinterpret_cast< const QModelIndex(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< QModelIndex*>(_a[0]) = std::move(_r); }  break;
        case 31: { QModelIndex _r = _t->sibling((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< const QModelIndex(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< QModelIndex*>(_a[0]) = std::move(_r); }  break;
        case 32: { int _r = _t->rowCount((*reinterpret_cast< const QModelIndex(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 33: { int _r = _t->rowCount();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 34: { int _r = _t->columnCount((*reinterpret_cast< const QModelIndex(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 35: { int _r = _t->columnCount();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 36: { bool _r = _t->hasChildren((*reinterpret_cast< const QModelIndex(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 37: { bool _r = _t->hasChildren();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 38: { QVariant _r = _t->data((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< QVariant*>(_a[0]) = std::move(_r); }  break;
        case 39: { QVariant _r = _t->data((*reinterpret_cast< const QModelIndex(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< QVariant*>(_a[0]) = std::move(_r); }  break;
        case 40: { bool _r = _t->setData((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< const QVariant(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 41: { bool _r = _t->setData((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< const QVariant(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 42: { QVariant _r = _t->headerData((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< Qt::Orientation(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< QVariant*>(_a[0]) = std::move(_r); }  break;
        case 43: { QVariant _r = _t->headerData((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< Qt::Orientation(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< QVariant*>(_a[0]) = std::move(_r); }  break;
        case 44: _t->fetchMore((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 45: { bool _r = _t->canFetchMore((*reinterpret_cast< const QModelIndex(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 46: { Qt::ItemFlags _r = _t->flags((*reinterpret_cast< const QModelIndex(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< Qt::ItemFlags*>(_a[0]) = std::move(_r); }  break;
        case 47: { QModelIndexList _r = _t->match((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< const QVariant(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4])),(*reinterpret_cast< Qt::MatchFlags(*)>(_a[5])));
            if (_a[0]) *reinterpret_cast< QModelIndexList*>(_a[0]) = std::move(_r); }  break;
        case 48: { QModelIndexList _r = _t->match((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< const QVariant(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< QModelIndexList*>(_a[0]) = std::move(_r); }  break;
        case 49: { QModelIndexList _r = _t->match((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< const QVariant(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< QModelIndexList*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 2:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QVector<int> >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QAbstractItemModel::*)(const QModelIndex & , const QModelIndex & , const QVector<int> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::dataChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(Qt::Orientation , int , int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::headerDataChanged)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(const QList<QPersistentModelIndex> & , QAbstractItemModel::LayoutChangeHint );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::layoutChanged)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(const QList<QPersistentModelIndex> & , QAbstractItemModel::LayoutChangeHint );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::layoutAboutToBeChanged)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(const QModelIndex & , int , int , QPrivateSignal);
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::rowsAboutToBeInserted)) {
                *result = 9;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(const QModelIndex & , int , int , QPrivateSignal);
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::rowsInserted)) {
                *result = 10;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(const QModelIndex & , int , int , QPrivateSignal);
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::rowsAboutToBeRemoved)) {
                *result = 11;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(const QModelIndex & , int , int , QPrivateSignal);
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::rowsRemoved)) {
                *result = 12;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(const QModelIndex & , int , int , QPrivateSignal);
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::columnsAboutToBeInserted)) {
                *result = 13;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(const QModelIndex & , int , int , QPrivateSignal);
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::columnsInserted)) {
                *result = 14;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(const QModelIndex & , int , int , QPrivateSignal);
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::columnsAboutToBeRemoved)) {
                *result = 15;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(const QModelIndex & , int , int , QPrivateSignal);
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::columnsRemoved)) {
                *result = 16;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(QPrivateSignal);
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::modelAboutToBeReset)) {
                *result = 17;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(QPrivateSignal);
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::modelReset)) {
                *result = 18;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(const QModelIndex & , int , int , const QModelIndex & , int , QPrivateSignal);
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::rowsAboutToBeMoved)) {
                *result = 19;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(const QModelIndex & , int , int , const QModelIndex & , int , QPrivateSignal);
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::rowsMoved)) {
                *result = 20;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(const QModelIndex & , int , int , const QModelIndex & , int , QPrivateSignal);
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::columnsAboutToBeMoved)) {
                *result = 21;
                return;
            }
        }
        {
            using _t = void (QAbstractItemModel::*)(const QModelIndex & , int , int , const QModelIndex & , int , QPrivateSignal);
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractItemModel::columnsMoved)) {
                *result = 22;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject QAbstractItemModel::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_QAbstractItemModel.data,
    qt_meta_data_QAbstractItemModel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QAbstractItemModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QAbstractItemModel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QAbstractItemModel.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int QAbstractItemModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 50)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 50;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 50)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 50;
    }
    return _id;
}

// SIGNAL 0
void QAbstractItemModel::dataChanged(const QModelIndex & _t1, const QModelIndex & _t2, const QVector<int> & _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 2
void QAbstractItemModel::headerDataChanged(Qt::Orientation _t1, int _t2, int _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void QAbstractItemModel::layoutChanged(const QList<QPersistentModelIndex> & _t1, QAbstractItemModel::LayoutChangeHint _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 6
void QAbstractItemModel::layoutAboutToBeChanged(const QList<QPersistentModelIndex> & _t1, QAbstractItemModel::LayoutChangeHint _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 9
void QAbstractItemModel::rowsAboutToBeInserted(const QModelIndex & _t1, int _t2, int _t3, QPrivateSignal _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void QAbstractItemModel::rowsInserted(const QModelIndex & _t1, int _t2, int _t3, QPrivateSignal _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void QAbstractItemModel::rowsAboutToBeRemoved(const QModelIndex & _t1, int _t2, int _t3, QPrivateSignal _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void QAbstractItemModel::rowsRemoved(const QModelIndex & _t1, int _t2, int _t3, QPrivateSignal _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 12, _a);
}

// SIGNAL 13
void QAbstractItemModel::columnsAboutToBeInserted(const QModelIndex & _t1, int _t2, int _t3, QPrivateSignal _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 13, _a);
}

// SIGNAL 14
void QAbstractItemModel::columnsInserted(const QModelIndex & _t1, int _t2, int _t3, QPrivateSignal _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 14, _a);
}

// SIGNAL 15
void QAbstractItemModel::columnsAboutToBeRemoved(const QModelIndex & _t1, int _t2, int _t3, QPrivateSignal _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 15, _a);
}

// SIGNAL 16
void QAbstractItemModel::columnsRemoved(const QModelIndex & _t1, int _t2, int _t3, QPrivateSignal _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 16, _a);
}

// SIGNAL 17
void QAbstractItemModel::modelAboutToBeReset(QPrivateSignal _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 17, _a);
}

// SIGNAL 18
void QAbstractItemModel::modelReset(QPrivateSignal _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 18, _a);
}

// SIGNAL 19
void QAbstractItemModel::rowsAboutToBeMoved(const QModelIndex & _t1, int _t2, int _t3, const QModelIndex & _t4, int _t5, QPrivateSignal _t6)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)), const_cast<void*>(reinterpret_cast<const void*>(&_t5)), const_cast<void*>(reinterpret_cast<const void*>(&_t6)) };
    QMetaObject::activate(this, &staticMetaObject, 19, _a);
}

// SIGNAL 20
void QAbstractItemModel::rowsMoved(const QModelIndex & _t1, int _t2, int _t3, const QModelIndex & _t4, int _t5, QPrivateSignal _t6)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)), const_cast<void*>(reinterpret_cast<const void*>(&_t5)), const_cast<void*>(reinterpret_cast<const void*>(&_t6)) };
    QMetaObject::activate(this, &staticMetaObject, 20, _a);
}

// SIGNAL 21
void QAbstractItemModel::columnsAboutToBeMoved(const QModelIndex & _t1, int _t2, int _t3, const QModelIndex & _t4, int _t5, QPrivateSignal _t6)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)), const_cast<void*>(reinterpret_cast<const void*>(&_t5)), const_cast<void*>(reinterpret_cast<const void*>(&_t6)) };
    QMetaObject::activate(this, &staticMetaObject, 21, _a);
}

// SIGNAL 22
void QAbstractItemModel::columnsMoved(const QModelIndex & _t1, int _t2, int _t3, const QModelIndex & _t4, int _t5, QPrivateSignal _t6)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)), const_cast<void*>(reinterpret_cast<const void*>(&_t5)), const_cast<void*>(reinterpret_cast<const void*>(&_t6)) };
    QMetaObject::activate(this, &staticMetaObject, 22, _a);
}
struct qt_meta_stringdata_QAbstractTableModel_t {
    QByteArrayData data[1];
    char stringdata0[20];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QAbstractTableModel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QAbstractTableModel_t qt_meta_stringdata_QAbstractTableModel = {
    {
QT_MOC_LITERAL(0, 0, 19) // "QAbstractTableModel"

    },
    "QAbstractTableModel"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QAbstractTableModel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void QAbstractTableModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject QAbstractTableModel::staticMetaObject = { {
    &QAbstractItemModel::staticMetaObject,
    qt_meta_stringdata_QAbstractTableModel.data,
    qt_meta_data_QAbstractTableModel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QAbstractTableModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QAbstractTableModel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QAbstractTableModel.stringdata0))
        return static_cast<void*>(this);
    return QAbstractItemModel::qt_metacast(_clname);
}

int QAbstractTableModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QAbstractItemModel::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_QAbstractListModel_t {
    QByteArrayData data[1];
    char stringdata0[19];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QAbstractListModel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QAbstractListModel_t qt_meta_stringdata_QAbstractListModel = {
    {
QT_MOC_LITERAL(0, 0, 18) // "QAbstractListModel"

    },
    "QAbstractListModel"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QAbstractListModel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void QAbstractListModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject QAbstractListModel::staticMetaObject = { {
    &QAbstractItemModel::staticMetaObject,
    qt_meta_stringdata_QAbstractListModel.data,
    qt_meta_data_QAbstractListModel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QAbstractListModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QAbstractListModel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QAbstractListModel.stringdata0))
        return static_cast<void*>(this);
    return QAbstractItemModel::qt_metacast(_clname);
}

int QAbstractListModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QAbstractItemModel::qt_metacall(_c, _id, _a);
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
