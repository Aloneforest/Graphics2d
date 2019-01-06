/********************************************************************************
** Form generated from reading UI file 'Graphics2d.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GRAPHICS2D_H
#define UI_GRAPHICS2D_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "imp2d.h"

QT_BEGIN_NAMESPACE

class Ui_Graphics2dClass
{
public:
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QLineEdit *console;
    Imp2d *imp;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *Graphics2dClass)
    {
        if (Graphics2dClass->objectName().isEmpty())
            Graphics2dClass->setObjectName(QStringLiteral("Graphics2dClass"));
        Graphics2dClass->resize(600, 400);
        centralWidget = new QWidget(Graphics2dClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout_2 = new QVBoxLayout(centralWidget);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        console = new QLineEdit(centralWidget);
        console->setObjectName(QStringLiteral("console"));

        verticalLayout->addWidget(console);

        imp = new Imp2d(centralWidget);
        imp->setObjectName(QStringLiteral("imp"));

        verticalLayout->addWidget(imp);


        verticalLayout_2->addLayout(verticalLayout);

        Graphics2dClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(Graphics2dClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 23));
        Graphics2dClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(Graphics2dClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        Graphics2dClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(Graphics2dClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        Graphics2dClass->setStatusBar(statusBar);

        retranslateUi(Graphics2dClass);

        QMetaObject::connectSlotsByName(Graphics2dClass);
    } // setupUi

    void retranslateUi(QMainWindow *Graphics2dClass)
    {
        Graphics2dClass->setWindowTitle(QApplication::translate("Graphics2dClass", "Graphics2d", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class Graphics2dClass: public Ui_Graphics2dClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GRAPHICS2D_H
