#include "j_state_pub.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    j_state_pub w;
    w.setGeometry(QRect(100,100,800,800));
    w.show();

    return a.exec();
}
