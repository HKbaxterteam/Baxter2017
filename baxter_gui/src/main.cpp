#include <ros/ros.h>
#include "baxter_gui.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Baxter_gui w;
    w.show();

    return a.exec();
}
