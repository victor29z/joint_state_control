#ifndef J_STATE_PUB_H
#define J_STATE_PUB_H

#include <QMainWindow>
#include <QLabel>
#include <QDebug>
#include <QTimer>
#include <QLineEdit>
#include <QSlider>
#include <QLayout>
#include <QCheckBox>

#include <QFile>
#include <QUdpSocket>

#include "joint_data_type.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Dense>
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
    QLineEdit *le_jangle[14];
    QLineEdit *le_dhangle[14];
    QLineEdit *lineEdit[14];
    QSlider *slider[14];
    QCheckBox *cbox[14];
    QCheckBox *cbox_select_all;
    QHBoxLayout *hLayout[14];
    QHBoxLayout *h_LbLayout[14];
    QVBoxLayout *vLayout[2];
    QHBoxLayout *mainLayout;

    QTimer *timer;

    ros::Publisher jointstates_publisher;

    /*  for data from master*/
    JOINT_DAT_TYPE joint_data;
    QUdpSocket* fromMasterHand;
    QFile *cfg;
    unsigned int cfg_UploadID_list[16];
    unsigned int cfg_DownloadID_list[16];
    unsigned int cfg_Data_offset[16];
    void get_configuration(void);

    double j_angle[16],dh_angle[16];
    double solved_angle[16];

    QFile *dat_log;


private slots:
	void timer_out(void);
    void slave_hand_recv();
    void select_all_checked(bool);

public:

    char master_addr[20];
};

#endif // J_STATE_PUB_H
