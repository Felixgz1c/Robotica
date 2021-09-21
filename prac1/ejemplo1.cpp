#include "ejemplo1.h"

/***
* Se hacen los connect para la interfaz de usuario con todas las funcionalidades creadas en el QTDesigner
 * e implementadas en código aquí
*/
ejemplo1::ejemplo1(): Ui_Counter()
{

	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()));
    connect(periodoMas, SIGNAL(clicked()), this, SLOT(incPeriod()));
    connect(periodoMenos, SIGNAL(clicked()), this, SLOT(decPeriod()));
	mytimer.connect(std::bind(&ejemplo1::showPeriod, this));
	mytimer.connect(std::bind(&ejemplo1::cuenta, this));
    time2.connect(std::bind(&ejemplo1::showTime, this));
    mytimer.start(period);
    time2.start(1000);
}

ejemplo1::~ejemplo1()
{}

/***
* Implementación de la función del botón de STOP
*/
void ejemplo1::doButton()
{
	static bool stopped = false;
	stopped = !stopped;
	if(stopped)
		mytimer.stop();
	else
		mytimer.start(period);
	qDebug() << "click on button";
}

/***
* Implementación de la función de mostrar el periodo del tick
*/
void ejemplo1::showPeriod()
{
    periodNumber->display(period);
}
/***
 * Implementación de la función de mostrar el tiempo transcurrido total
 */
void ejemplo1::showTime()
{
    timeNumber->display(++timecont);
}

/***
* Implementación de la función de aumentar el periodo del tick
*/
void ejemplo1::incPeriod()
{
    //periodNumber->display(period+500);
    period += 500;
    mytimer.setPeriod(period);
    showPeriod();
}

/***
* Implementación de la función de disminuir el periodo del tick
*/
void ejemplo1::decPeriod() {
    if (period > 500) {
        //periodNumber->display(period - 500);
        period -=500;
        mytimer.setPeriod(period);
    }
    showPeriod();

}

/***
* Implementación de la función de mostrar la cuenta de ticks
*/
void ejemplo1::cuenta()
{
    lcdNumber->display(++cont);
	//trick++;
}

