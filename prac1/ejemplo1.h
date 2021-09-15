#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include <chrono>

class QTimer;

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();
        virtual ~ejemplo1();

    public slots:
        void doButton();

    private:
        QTimer *m_timer;

        int cont=0;

        void cuenta();

        int trick= 5;
};

#endif // ejemplo1_H
