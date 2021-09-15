#include "ejemplo1.h"
#include <QTimer>

ejemplo1::ejemplo1(): Ui_Counter()
{
    m_timer = new QTimer(this);
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
    //m_timer->connect();
    m_timer->start(500);
}

ejemplo1::~ejemplo1()
{}

void ejemplo1::doButton()
{
    static bool stopped = false;
    stopped = !stopped;
    if(stopped)
        m_timer->stop();
    else
        m_timer->start(500);
    qDebug() << "click on button";
}

void ejemplo1::cuenta()
{
    lcdNumber->display(++cont);
    trick++;
}





