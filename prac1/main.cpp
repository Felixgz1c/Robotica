#include <QApplication>
#include "ejemplo1.h"

/***
 * AUTORES : JAIME GONZÁLEZ NAVAREÑO Y FÉLIX GONZÁLEZ ZAERA
 */


int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    ejemplo1 foo;
    foo.show();
    return app.exec();
}
