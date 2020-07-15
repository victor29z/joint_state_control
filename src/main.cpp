#include "j_state_pub.h"
#include <QApplication>
#include <iostream>
#include <QDir>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    j_state_pub w;
    QDir::setCurrent(a.applicationDirPath());
    std::cout << "arc="<<argc<<std::endl;
    //std::cout << "argv[1]="<<argv[1]<<std::endl;
    if (argc >1 )
        strcpy(w.master_addr,argv[1]);
    //std::cout << "argv[1]="<<w.new_addr<<std::endl;
    w.setGeometry(QRect(100,100,800,800));
    w.show();

    return a.exec();
}
