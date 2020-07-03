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
#include <QString>
#include <QFile>
#include <QUdpSocket>

#include "joint_data_type.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Dense>
#define NO_OF_JOINTS 20
#define DOF_EACH_ARM 9
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
    QLabel *lbl[NO_OF_JOINTS];
    QLineEdit *le_jangle[NO_OF_JOINTS];
    QLineEdit *le_dhangle[NO_OF_JOINTS];
    QLineEdit *lineEdit[NO_OF_JOINTS];
    QSlider *slider[NO_OF_JOINTS];
    QCheckBox *cbox[NO_OF_JOINTS];
    QCheckBox *cbox_select_all;
    QHBoxLayout *hLayout[NO_OF_JOINTS];
    QHBoxLayout *h_LbLayout[NO_OF_JOINTS];
    QVBoxLayout *vLayout[NO_OF_JOINTS];
    QVBoxLayout *mainLayout;
    QHBoxLayout *body_control_Layout; // use to put 2 body control slider
    QHBoxLayout *arm_asm_Layout;// use to put left and right arm
    QTimer *timer;

    ros::Publisher jointstates_publisher;

    /*  for data from master*/
    JOINT_DAT_TYPE joint_data;
    QUdpSocket* fromMasterHand;
    QUdpSocket* toSlaveSock;
    QFile *cfg;
    unsigned int cfg_UploadID_list[16];
    unsigned int cfg_DownloadID_list[16];
    unsigned int cfg_Data_offset[16];
    void get_configuration(void);

    double j_angle[16],dh_angle[16];
    double solved_angle[16];

    QFile *dat_log;

    sensor_msgs::JointState joint_state;
    HANDLE_DAT_TYPE handle_data;

    float waist_lr,waist_ud;




private slots:
	void timer_out(void);
    void slave_hand_recv();
    void select_all_checked(bool);

public:

    char master_addr[20];
    void Joint_State_Msg_Initialize(int size, char* joint_name_list[]);
    void setup_ui(void);
};

#endif // J_STATE_PUB_H
