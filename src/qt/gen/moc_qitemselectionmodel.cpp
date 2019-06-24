/****************************************************************************
** Meta object code from reading C++ file 'qitemselectionmodel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/qbase/src/corelib/itemmodels/qitemselectionmodel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qitemselectionmodel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QItemSelectionModel_t {
    QByteArrayData data[62];
    char stringdata0[876];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QItemSelectionModel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QItemSelectionModel_t qt_meta_stringdata_QItemSelectionModel = {
    {
QT_MOC_LITERAL(0, 0, 19), // "QItemSelectionModel"
QT_MOC_LITERAL(1, 20, 16), // "selectionChanged"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 14), // "QItemSelection"
QT_MOC_LITERAL(4, 53, 8), // "selected"
QT_MOC_LITERAL(5, 62, 10), // "deselected"
QT_MOC_LITERAL(6, 73, 14), // "currentChanged"
QT_MOC_LITERAL(7, 88, 11), // "QModelIndex"
QT_MOC_LITERAL(8, 100, 7), // "current"
QT_MOC_LITERAL(9, 108, 8), // "previous"
QT_MOC_LITERAL(10, 117, 17), // "currentRowChanged"
QT_MOC_LITERAL(11, 135, 20), // "currentColumnChanged"
QT_MOC_LITERAL(12, 156, 12), // "modelChanged"
QT_MOC_LITERAL(13, 169, 19), // "QAbstractItemModel*"
QT_MOC_LITERAL(14, 189, 5), // "model"
QT_MOC_LITERAL(15, 195, 15), // "setCurrentIndex"
QT_MOC_LITERAL(16, 211, 5), // "index"
QT_MOC_LITERAL(17, 217, 35), // "QItemSelectionModel::Selectio..."
QT_MOC_LITERAL(18, 253, 7), // "command"
QT_MOC_LITERAL(19, 261, 6), // "select"
QT_MOC_LITERAL(20, 268, 9), // "selection"
QT_MOC_LITERAL(21, 278, 5), // "clear"
QT_MOC_LITERAL(22, 284, 5), // "reset"
QT_MOC_LITERAL(23, 290, 14), // "clearSelection"
QT_MOC_LITERAL(24, 305, 17), // "clearCurrentIndex"
QT_MOC_LITERAL(25, 323, 26), // "_q_columnsAboutToBeRemoved"
QT_MOC_LITERAL(26, 350, 23), // "_q_rowsAboutToBeRemoved"
QT_MOC_LITERAL(27, 374, 27), // "_q_columnsAboutToBeInserted"
QT_MOC_LITERAL(28, 402, 24), // "_q_rowsAboutToBeInserted"
QT_MOC_LITERAL(29, 427, 25), // "_q_layoutAboutToBeChanged"
QT_MOC_LITERAL(30, 453, 28), // "QList<QPersistentModelIndex>"
QT_MOC_LITERAL(31, 482, 7), // "parents"
QT_MOC_LITERAL(32, 490, 36), // "QAbstractItemModel::LayoutCha..."
QT_MOC_LITERAL(33, 527, 4), // "hint"
QT_MOC_LITERAL(34, 532, 16), // "_q_layoutChanged"
QT_MOC_LITERAL(35, 549, 10), // "isSelected"
QT_MOC_LITERAL(36, 560, 13), // "isRowSelected"
QT_MOC_LITERAL(37, 574, 3), // "row"
QT_MOC_LITERAL(38, 578, 6), // "parent"
QT_MOC_LITERAL(39, 585, 16), // "isColumnSelected"
QT_MOC_LITERAL(40, 602, 6), // "column"
QT_MOC_LITERAL(41, 609, 22), // "rowIntersectsSelection"
QT_MOC_LITERAL(42, 632, 25), // "columnIntersectsSelection"
QT_MOC_LITERAL(43, 658, 12), // "selectedRows"
QT_MOC_LITERAL(44, 671, 15), // "QModelIndexList"
QT_MOC_LITERAL(45, 687, 15), // "selectedColumns"
QT_MOC_LITERAL(46, 703, 12), // "hasSelection"
QT_MOC_LITERAL(47, 716, 12), // "currentIndex"
QT_MOC_LITERAL(48, 729, 15), // "selectedIndexes"
QT_MOC_LITERAL(49, 745, 14), // "SelectionFlags"
QT_MOC_LITERAL(50, 760, 13), // "SelectionFlag"
QT_MOC_LITERAL(51, 774, 8), // "NoUpdate"
QT_MOC_LITERAL(52, 783, 5), // "Clear"
QT_MOC_LITERAL(53, 789, 6), // "Select"
QT_MOC_LITERAL(54, 796, 8), // "Deselect"
QT_MOC_LITERAL(55, 805, 6), // "Toggle"
QT_MOC_LITERAL(56, 812, 7), // "Current"
QT_MOC_LITERAL(57, 820, 4), // "Rows"
QT_MOC_LITERAL(58, 825, 7), // "Columns"
QT_MOC_LITERAL(59, 833, 13), // "SelectCurrent"
QT_MOC_LITERAL(60, 847, 13), // "ToggleCurrent"
QT_MOC_LITERAL(61, 861, 14) // "ClearAndSelect"

    },
    "QItemSelectionModel\0selectionChanged\0"
    "\0QItemSelection\0selected\0deselected\0"
    "currentChanged\0QModelIndex\0current\0"
    "previous\0currentRowChanged\0"
    "currentColumnChanged\0modelChanged\0"
    "QAbstractItemModel*\0model\0setCurrentIndex\0"
    "index\0QItemSelectionModel::SelectionFlags\0"
    "command\0select\0selection\0clear\0reset\0"
    "clearSelection\0clearCurrentIndex\0"
    "_q_columnsAboutToBeRemoved\0"
    "_q_rowsAboutToBeRemoved\0"
    "_q_columnsAboutToBeInserted\0"
    "_q_rowsAboutToBeInserted\0"
    "_q_layoutAboutToBeChanged\0"
    "QList<QPersistentModelIndex>\0parents\0"
    "QAbstractItemModel::LayoutChangeHint\0"
    "hint\0_q_layoutChanged\0isSelected\0"
    "isRowSelected\0row\0parent\0isColumnSelected\0"
    "column\0rowIntersectsSelection\0"
    "columnIntersectsSelection\0selectedRows\0"
    "QModelIndexList\0selectedColumns\0"
    "hasSelection\0currentIndex\0selectedIndexes\0"
    "SelectionFlags\0SelectionFlag\0NoUpdate\0"
    "Clear\0Select\0Deselect\0Toggle\0Current\0"
    "Rows\0Columns\0SelectCurrent\0ToggleCurrent\0"
    "ClearAndSelect"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QItemSelectionModel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      31,   14, // methods
       5,  288, // properties
       1,  308, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,  169,    2, 0x06 /* Public */,
       6,    2,  174,    2, 0x06 /* Public */,
      10,    2,  179,    2, 0x06 /* Public */,
      11,    2,  184,    2, 0x06 /* Public */,
      12,    1,  189,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      15,    2,  192,    2, 0x0a /* Public */,
      19,    2,  197,    2, 0x0a /* Public */,
      19,    2,  202,    2, 0x0a /* Public */,
      21,    0,  207,    2, 0x0a /* Public */,
      22,    0,  208,    2, 0x0a /* Public */,
      23,    0,  209,    2, 0x0a /* Public */,
      24,    0,  210,    2, 0x0a /* Public */,
      25,    3,  211,    2, 0x08 /* Private */,
      26,    3,  218,    2, 0x08 /* Private */,
      27,    3,  225,    2, 0x08 /* Private */,
      28,    3,  232,    2, 0x08 /* Private */,
      29,    2,  239,    2, 0x08 /* Private */,
      29,    1,  244,    2, 0x28 /* Private | MethodCloned */,
      29,    0,  247,    2, 0x28 /* Private | MethodCloned */,
      34,    2,  248,    2, 0x08 /* Private */,
      34,    1,  253,    2, 0x28 /* Private | MethodCloned */,
      34,    0,  256,    2, 0x28 /* Private | MethodCloned */,

 // methods: name, argc, parameters, tag, flags
      35,    1,  257,    2, 0x02 /* Public */,
      36,    2,  260,    2, 0x02 /* Public */,
      39,    2,  265,    2, 0x02 /* Public */,
      41,    2,  270,    2, 0x02 /* Public */,
      42,    2,  275,    2, 0x02 /* Public */,
      43,    1,  280,    2, 0x02 /* Public */,
      43,    0,  283,    2, 0x22 /* Public | MethodCloned */,
      45,    1,  284,    2, 0x02 /* Public */,
      45,    0,  287,    2, 0x22 /* Public | MethodCloned */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 3,    4,    5,
    QMetaType::Void, 0x80000000 | 7, 0x80000000 | 7,    8,    9,
    QMetaType::Void, 0x80000000 | 7, 0x80000000 | 7,    8,    9,
    QMetaType::Void, 0x80000000 | 7, 0x80000000 | 7,    8,    9,
    QMetaType::Void, 0x80000000 | 13,   14,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 7, 0x80000000 | 17,   16,   18,
    QMetaType::Void, 0x80000000 | 7, 0x80000000 | 17,   16,   18,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 17,   20,   18,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 7, QMetaType::Int, QMetaType::Int,    2,    2,    2,
    QMetaType::Void, 0x80000000 | 7, QMetaType::Int, QMetaType::Int,    2,    2,    2,
    QMetaType::Void, 0x80000000 | 7, QMetaType::Int, QMetaType::Int,    2,    2,    2,
    QMetaType::Void, 0x80000000 | 7, QMetaType::Int, QMetaType::Int,    2,    2,    2,
    QMetaType::Void, 0x80000000 | 30, 0x80000000 | 32,   31,   33,
    QMetaType::Void, 0x80000000 | 30,   31,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 30, 0x80000000 | 32,   31,   33,
    QMetaType::Void, 0x80000000 | 30,   31,
    QMetaType::Void,

 // methods: parameters
    QMetaType::Bool, 0x80000000 | 7,   16,
    QMetaType::Bool, QMetaType::Int, 0x80000000 | 7,   37,   38,
    QMetaType::Bool, QMetaType::Int, 0x80000000 | 7,   40,   38,
    QMetaType::Bool, QMetaType::Int, 0x80000000 | 7,   37,   38,
    QMetaType::Bool, QMetaType::Int, 0x80000000 | 7,   40,   38,
    0x80000000 | 44, QMetaType::Int,   40,
    0x80000000 | 44,
    0x80000000 | 44, QMetaType::Int,   37,
    0x80000000 | 44,

 // properties: name, type, flags
      14, 0x80000000 | 13, 0x0049510b,
      46, QMetaType::Bool, 0x00484001,
      47, 0x80000000 | 7, 0x00484009,
      20, 0x80000000 | 3, 0x00484009,
      48, 0x80000000 | 44, 0x00484009,

 // properties: notify_signal_id
       4,
       0,
       1,
       0,
       0,

 // enums: name, alias, flags, count, data
      49,   50, 0x1,   11,  313,

 // enum data: key, value
      51, uint(QItemSelectionModel::NoUpdate),
      52, uint(QItemSelectionModel::Clear),
      53, uint(QItemSelectionModel::Select),
      54, uint(QItemSelectionModel::Deselect),
      55, uint(QItemSelectionModel::Toggle),
      56, uint(QItemSelectionModel::Current),
      57, uint(QItemSelectionModel::Rows),
      58, uint(QItemSelectionModel::Columns),
      59, uint(QItemSelectionModel::SelectCurrent),
      60, uint(QItemSelectionModel::ToggleCurrent),
      61, uint(QItemSelectionModel::ClearAndSelect),

       0        // eod
};

void QItemSelectionModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QItemSelectionModel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->selectionChanged((*reinterpret_cast< const QItemSelection(*)>(_a[1])),(*reinterpret_cast< const QItemSelection(*)>(_a[2]))); break;
        case 1: _t->currentChanged((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< const QModelIndex(*)>(_a[2]))); break;
        case 2: _t->currentRowChanged((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< const QModelIndex(*)>(_a[2]))); break;
        case 3: _t->currentColumnChanged((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< const QModelIndex(*)>(_a[2]))); break;
        case 4: _t->modelChanged((*reinterpret_cast< QAbstractItemModel*(*)>(_a[1]))); break;
        case 5: _t->setCurrentIndex((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< QItemSelectionModel::SelectionFlags(*)>(_a[2]))); break;
        case 6: _t->select((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< QItemSelectionModel::SelectionFlags(*)>(_a[2]))); break;
        case 7: _t->select((*reinterpret_cast< const QItemSelection(*)>(_a[1])),(*reinterpret_cast< QItemSelectionModel::SelectionFlags(*)>(_a[2]))); break;
        case 8: _t->clear(); break;
        case 9: _t->reset(); break;
        case 10: _t->clearSelection(); break;
        case 11: _t->clearCurrentIndex(); break;
        case 12: _t->d_func()->_q_columnsAboutToBeRemoved((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 13: _t->d_func()->_q_rowsAboutToBeRemoved((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 14: _t->d_func()->_q_columnsAboutToBeInserted((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 15: _t->d_func()->_q_rowsAboutToBeInserted((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 16: _t->d_func()->_q_layoutAboutToBeChanged((*reinterpret_cast< const QList<QPersistentModelIndex>(*)>(_a[1])),(*reinterpret_cast< QAbstractItemModel::LayoutChangeHint(*)>(_a[2]))); break;
        case 17: _t->d_func()->_q_layoutAboutToBeChanged((*reinterpret_cast< const QList<QPersistentModelIndex>(*)>(_a[1]))); break;
        case 18: _t->d_func()->_q_layoutAboutToBeChanged(); break;
        case 19: _t->d_func()->_q_layoutChanged((*reinterpret_cast< const QList<QPersistentModelIndex>(*)>(_a[1])),(*reinterpret_cast< QAbstractItemModel::LayoutChangeHint(*)>(_a[2]))); break;
        case 20: _t->d_func()->_q_layoutChanged((*reinterpret_cast< const QList<QPersistentModelIndex>(*)>(_a[1]))); break;
        case 21: _t->d_func()->_q_layoutChanged(); break;
        case 22: { bool _r = _t->isSelected((*reinterpret_cast< const QModelIndex(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 23: { bool _r = _t->isRowSelected((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< const QModelIndex(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 24: { bool _r = _t->isColumnSelected((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< const QModelIndex(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 25: { bool _r = _t->rowIntersectsSelection((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< const QModelIndex(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 26: { bool _r = _t->columnIntersectsSelection((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< const QModelIndex(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 27: { QModelIndexList _r = _t->selectedRows((*reinterpret_cast< int(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< QModelIndexList*>(_a[0]) = std::move(_r); }  break;
        case 28: { QModelIndexList _r = _t->selectedRows();
            if (_a[0]) *reinterpret_cast< QModelIndexList*>(_a[0]) = std::move(_r); }  break;
        case 29: { QModelIndexList _r = _t->selectedColumns((*reinterpret_cast< int(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< QModelIndexList*>(_a[0]) = std::move(_r); }  break;
        case 30: { QModelIndexList _r = _t->selectedColumns();
            if (_a[0]) *reinterpret_cast< QModelIndexList*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QItemSelection >(); break;
            }
            break;
        case 7:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QItemSelection >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QItemSelectionModel::*)(const QItemSelection & , const QItemSelection & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QItemSelectionModel::selectionChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (QItemSelectionModel::*)(const QModelIndex & , const QModelIndex & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QItemSelectionModel::currentChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (QItemSelectionModel::*)(const QModelIndex & , const QModelIndex & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QItemSelectionModel::currentRowChanged)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (QItemSelectionModel::*)(const QModelIndex & , const QModelIndex & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QItemSelectionModel::currentColumnChanged)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (QItemSelectionModel::*)(QAbstractItemModel * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QItemSelectionModel::modelChanged)) {
                *result = 4;
                return;
            }
        }
    } else if (_c == QMetaObject::RegisterPropertyMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 3:
            *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QItemSelection >(); break;
        }
    }

#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty) {
        auto *_t = static_cast<QItemSelectionModel *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< QAbstractItemModel**>(_v) = _t->model(); break;
        case 1: *reinterpret_cast< bool*>(_v) = _t->hasSelection(); break;
        case 2: *reinterpret_cast< QModelIndex*>(_v) = _t->currentIndex(); break;
        case 3: *reinterpret_cast< QItemSelection*>(_v) = _t->selection(); break;
        case 4: *reinterpret_cast< QModelIndexList*>(_v) = _t->selectedIndexes(); break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        auto *_t = static_cast<QItemSelectionModel *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: _t->setModel(*reinterpret_cast< QAbstractItemModel**>(_v)); break;
        default: break;
        }
    } else if (_c == QMetaObject::ResetProperty) {
    }
#endif // QT_NO_PROPERTIES
}

QT_INIT_METAOBJECT const QMetaObject QItemSelectionModel::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_QItemSelectionModel.data,
    qt_meta_data_QItemSelectionModel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QItemSelectionModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QItemSelectionModel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QItemSelectionModel.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int QItemSelectionModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 31)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 31;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 31)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 31;
    }
#ifndef QT_NO_PROPERTIES
   else if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 5;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 5;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 5;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 5;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 5;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}

// SIGNAL 0
void QItemSelectionModel::selectionChanged(const QItemSelection & _t1, const QItemSelection & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void QItemSelectionModel::currentChanged(const QModelIndex & _t1, const QModelIndex & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void QItemSelectionModel::currentRowChanged(const QModelIndex & _t1, const QModelIndex & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void QItemSelectionModel::currentColumnChanged(const QModelIndex & _t1, const QModelIndex & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void QItemSelectionModel::modelChanged(QAbstractItemModel * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
