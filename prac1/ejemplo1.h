#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include <chrono>
#include "timer.h"

/***
* Se añadieron las cabeceras de los métodos nuevos así como las variables necesarias como timecont, period, etc
*/
class ejemplo1 : public QWidget, public Ui_Counter
{
Q_OBJECT
    public:
        ejemplo1();
        virtual ~ejemplo1();
    
    public slots:
	void doButton();
    void showPeriod();
    void showTime();
    void incPeriod();
    void decPeriod();

    private:
        Timer mytimer, mytimerLong;
        Timer time2;
        int cont = 0;
        int timecont = 0;
		int period = 500;
		// dos callbacks con diferente número de parámetros
        void cuenta();
		
//		int trick = 5;
};

#endif // ejemplo1_H
