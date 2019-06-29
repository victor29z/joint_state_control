#ifndef J_STATE_PUB_H
#define J_STATE_PUB_H

#include <QMainWindow>
#include <QLabel>
#include <QDebug>
#include <QTimer>
#include <QLineEdit>
#include <QSlider>
#include <QLayout>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace Ui {
class j_state_pub;
}

class j_state_pub : public QMainWindow
{
    Q_OBJECT

public:
    explicit j_state_pub(QWidget *parent = 0);
    ~j_state_pub();

private:
    Ui::j_state_pub *ui;
    QLabel *lbl[14];
    QLineEdit *lineEdit[14];
    QSlider *slider[14];
    QHBoxLayout *hLayout[14];
    QVBoxLayout *vLayout[2];
    QHBoxLayout *mainLayout;

    QTimer *timer;

    ros::Publisher jointstates_publisher;


private slots:
	void timer_out(void);
};

#endif // J_STATE_PUB_H
