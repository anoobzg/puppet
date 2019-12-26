/********************************************************************************
** Form generated from reading UI file 'Lasso.ui'
**
** Created by: Qt User Interface Compiler version 5.12.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LASSO_H
#define UI_LASSO_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LassoClass
{
public:
    QAction *actionINPUT;
    QAction *actionSEARCH;
    QWidget *centralWidget;
    QLabel *label;
    QLabel *label_2;
    QMenuBar *menuBar;
    QMenu *menuFILE;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *LassoClass)
    {
        if (LassoClass->objectName().isEmpty())
            LassoClass->setObjectName(QString::fromUtf8("LassoClass"));
        LassoClass->resize(1617, 861);
        actionINPUT = new QAction(LassoClass);
        actionINPUT->setObjectName(QString::fromUtf8("actionINPUT"));
        actionSEARCH = new QAction(LassoClass);
        actionSEARCH->setObjectName(QString::fromUtf8("actionSEARCH"));
        centralWidget = new QWidget(LassoClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 0, 50, 40));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);
        label->setMinimumSize(QSize(50, 40));
        label->setMaximumSize(QSize(50, 40));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(30, 50, 1080, 1080));
        sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy);
        label_2->setMinimumSize(QSize(1080, 1080));
        label_2->setMaximumSize(QSize(500, 400));
        LassoClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(LassoClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1617, 23));
        menuFILE = new QMenu(menuBar);
        menuFILE->setObjectName(QString::fromUtf8("menuFILE"));
        LassoClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(LassoClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        LassoClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(LassoClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        LassoClass->setStatusBar(statusBar);

        menuBar->addAction(menuFILE->menuAction());
        menuFILE->addAction(actionINPUT);
        menuFILE->addAction(actionSEARCH);

        retranslateUi(LassoClass);

        QMetaObject::connectSlotsByName(LassoClass);
    } // setupUi

    void retranslateUi(QMainWindow *LassoClass)
    {
        LassoClass->setWindowTitle(QApplication::translate("LassoClass", "Lasso", nullptr));
        actionINPUT->setText(QApplication::translate("LassoClass", "INPUT", nullptr));
        actionSEARCH->setText(QApplication::translate("LassoClass", "SEARCH", nullptr));
        label->setText(QString());
        label_2->setText(QString());
        menuFILE->setTitle(QApplication::translate("LassoClass", "FILE", nullptr));
    } // retranslateUi

};

namespace Ui {
    class LassoClass: public Ui_LassoClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LASSO_H
