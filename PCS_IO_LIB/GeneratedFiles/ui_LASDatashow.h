/********************************************************************************
** Form generated from reading UI file 'LASDatashow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LASDATASHOW_H
#define UI_LASDATASHOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LASDatashow
{
public:

    void setupUi(QWidget *LASDatashow)
    {
        if (LASDatashow->objectName().isEmpty())
            LASDatashow->setObjectName(QStringLiteral("LASDatashow"));
        LASDatashow->resize(400, 300);

        retranslateUi(LASDatashow);

        QMetaObject::connectSlotsByName(LASDatashow);
    } // setupUi

    void retranslateUi(QWidget *LASDatashow)
    {
        LASDatashow->setWindowTitle(QApplication::translate("LASDatashow", "LASDatashow", 0));
    } // retranslateUi

};

namespace Ui {
    class LASDatashow: public Ui_LASDatashow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LASDATASHOW_H
