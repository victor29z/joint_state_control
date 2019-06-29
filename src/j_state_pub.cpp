#include "j_state_pub.h"
#include "ui_j_state_pub.h"

j_state_pub::j_state_pub(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::j_state_pub)
{
    ui->setupUi(this);
    /*
     * ros initialize code
     *
     */
    int argc = 0; char **argv = NULL;
	ros::init(argc, argv, "client_plug");
	if (!ros::master::check())
	{
		ROS_INFO("No master started!");
		this->close();
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	jointstates_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1000);

    vLayout[0] = new QVBoxLayout();
    vLayout[1] = new QVBoxLayout();
    int i = 0;
    for(i = 0; i < 14; i++){
        hLayout[i] = new QHBoxLayout();
        lbl[i] = new QLabel(this);
        slider[i] = new QSlider(this);
        lineEdit[i] = new QLineEdit(this);
        QString s;

        if(i < 7){
            s.setNum(i);
            lbl[i]->setText(tr("L")+s);
        }
        else{
            s.setNum(i-7);
            lbl[i]->setText(tr("R")+s);
        }

        slider[i]->setOrientation(Qt::Horizontal);
        slider[i]->setMinimum(-628);
        slider[i]->setMaximum(628);
        slider[i]->setValue(0);
        lineEdit[i]->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
        lineEdit[i]->setFixedWidth(50);
        hLayout[i]->addWidget(lbl[i]);
        //hLayout->addStretch();
        hLayout[i]->addWidget(slider[i]);
        //hLayout->addStretch();
        hLayout[i]->addWidget(lineEdit[i]);

        if(i < 7){
            vLayout[0]->addLayout(hLayout[i]);
            vLayout[0]->addStretch();
        }
        else{
            vLayout[1]->addLayout(hLayout[i]);
            vLayout[1]->addStretch();
        }
    }
    mainLayout = new QHBoxLayout();
    mainLayout->addLayout(vLayout[0]);
    //mainLayout->addStretch();
    mainLayout->addLayout(vLayout[1]);




    ui->centralWidget->setLayout(mainLayout);

    timer = new QTimer(this);
    timer->setInterval(50);

    connect(timer,SIGNAL(timeout()),this,SLOT(timer_out()));
    timer->start();




}

j_state_pub::~j_state_pub()
{
    delete ui;
}

void j_state_pub::timer_out(void){
    int i;
    sensor_msgs::JointState joint_state;

	joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(14);
	joint_state.position.resize(14);
	joint_state.name[0] = "joint-l1";
	joint_state.name[1] = "joint-l2";
	joint_state.name[2] = "joint-l3";
	joint_state.name[3] = "joint-l4";
	joint_state.name[4] = "joint-l5";
	joint_state.name[5] = "joint-l6";
	joint_state.name[6] = "joint-l7";
	joint_state.name[7] = "joint-r1";
	joint_state.name[8] = "joint-r2";
	joint_state.name[9] = "joint-r3";
	joint_state.name[10] = "joint-r4";
	joint_state.name[11] = "joint-r5";
	joint_state.name[12] = "joint-r6";
	joint_state.name[13] = "joint-r7";

    for(i = 0; i < 14; i++){
        int tmp = slider[i]->value();
        QString s;
        s.setNum(tmp);
        lineEdit[i]->setText(s);
        joint_state.position[i] = tmp / 100.0;

    }
    jointstates_publisher.publish(joint_state);

}
