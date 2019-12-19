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
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LassoClass
{
public:
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QWidget *centralWidget;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *LassoClass)
    {
        if (LassoClass->objectName().isEmpty())
            LassoClass->setObjectName(QString::fromUtf8("LassoClass"));
        LassoClass->resize(600, 400);
        menuBar = new QMenuBar(LassoClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        LassoClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(LassoClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        LassoClass->addToolBar(mainToolBar);
        centralWidget = new QWidget(LassoClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        LassoClass->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(LassoClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        LassoClass->setStatusBar(statusBar);

        retranslateUi(LassoClass);

        QMetaObject::connectSlotsByName(LassoClass);
    } // setupUi

    void retranslateUi(QMainWindow *LassoClass)
    {
        LassoClass->setWindowTitle(QApplication::translate("LassoClass", "Lasso", nullptr));
    } // retranslateUi

};

namespace Ui {
    class LassoClass: public Ui_LassoClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LASSO_H
