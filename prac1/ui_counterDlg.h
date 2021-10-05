/********************************************************************************
** Form generated from reading UI file 'counterDlg.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_COUNTERDLG_H
#define UI_COUNTERDLG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Counter
{
public:
    QPushButton *button;
    QLCDNumber *lcdNumber;
    QLCDNumber *periodNumber;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLCDNumber *timeNumber;
    QPushButton *periodoMas;
    QPushButton *periodoMenos;

    void setupUi(QWidget *Counter)
    {
        if (Counter->objectName().isEmpty())
            Counter->setObjectName(QString::fromUtf8("Counter"));
        Counter->resize(575, 300);
        button = new QPushButton(Counter);
        button->setObjectName(QString::fromUtf8("button"));
        button->setGeometry(QRect(40, 170, 251, 71));
        lcdNumber = new QLCDNumber(Counter);
        lcdNumber->setObjectName(QString::fromUtf8("lcdNumber"));
        lcdNumber->setGeometry(QRect(0, 10, 191, 91));
        periodNumber = new QLCDNumber(Counter);
        periodNumber->setObjectName(QString::fromUtf8("periodNumber"));
        periodNumber->setGeometry(QRect(200, 10, 181, 91));
        label = new QLabel(Counter);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(260, 110, 91, 17));
        label_2 = new QLabel(Counter);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(50, 110, 91, 17));
        label_3 = new QLabel(Counter);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(440, 110, 91, 17));
        timeNumber = new QLCDNumber(Counter);
        timeNumber->setObjectName(QString::fromUtf8("timeNumber"));
        timeNumber->setGeometry(QRect(390, 10, 181, 91));
        periodoMas = new QPushButton(Counter);
        periodoMas->setObjectName(QString::fromUtf8("periodoMas"));
        periodoMas->setGeometry(QRect(318, 170, 231, 41));
        periodoMenos = new QPushButton(Counter);
        periodoMenos->setObjectName(QString::fromUtf8("periodoMenos"));
        periodoMenos->setGeometry(QRect(320, 214, 231, 41));

        retranslateUi(Counter);

        QMetaObject::connectSlotsByName(Counter);
    } // setupUi

    void retranslateUi(QWidget *Counter)
    {
        Counter->setWindowTitle(QApplication::translate("Counter", "Counter", nullptr));
        button->setText(QApplication::translate("Counter", "STOP", nullptr));
        label->setText(QApplication::translate("Counter", "Period in ms", nullptr));
        label_2->setText(QApplication::translate("Counter", "Tick counter", nullptr));
        label_3->setText(QApplication::translate("Counter", "Time counter", nullptr));
        periodoMas->setText(QApplication::translate("Counter", "Period +", nullptr));
        periodoMenos->setText(QApplication::translate("Counter", "Period -", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Counter: public Ui_Counter {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_COUNTERDLG_H
