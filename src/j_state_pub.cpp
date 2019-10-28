#include "j_state_pub.h"
#include "ui_j_state_pub.h"
#include <QtMath>
#include <iostream>
#define FROM_MASTER_HAND_PORT  12625
#define TO_MASTER_HAND_PORT 17362

using namespace Eigen;
char master_hand_addr[20] = "192.168.1.167";
const unsigned int enc_modula[16] = {
    131072, //L1
    131072, //L2
    131072, //L3
    131072, //L4
    524288, //L5
    4096, //L6
    4096, //L7
    1, //LH
    131072, //R1
    131072, //R2
    131072, //R3
    131072, //R4
    524288, //R5
    4096, //R6
    4096, //R7
    1 //RH


};

const float scale_factor[16] ={
    364.089,
    364.089,
    364.089,
    364.089,
    1456.356,
    11.378,
    11.378,
    1.0,

    364.089,
    364.089,
    364.089,
    364.089,
    1456.356,
    11.378,
    11.378,
    1.0

};

const int transfer_seq[14] = {8,9,10,11,12,13,14, 0,1,2,3,4,5,6};

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
        h_LbLayout[i] = new QHBoxLayout();
        lbl[i] = new QLabel(this);
        le_jangle[i] = new QLineEdit(this);
        le_dhangle[i] = new QLineEdit(this);
        slider[i] = new QSlider(this);
        lineEdit[i] = new QLineEdit(this);
        cbox[i] = new QCheckBox(this);
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
        le_jangle[i]->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
        le_jangle[i]->setFixedWidth(200);
        le_dhangle[i]->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
        le_dhangle[i]->setFixedWidth(200);
        hLayout[i]->addWidget(lbl[i]);
        //hLayout->addStretch();
        hLayout[i]->addWidget(slider[i]);
        //hLayout->addStretch();
        hLayout[i]->addWidget(lineEdit[i]);
        hLayout[i]->addWidget(cbox[i]);
        h_LbLayout[i]->addWidget(le_jangle[i]);
        h_LbLayout[i]->addWidget(le_dhangle[i]);
        if(i < 7){
            vLayout[0]->addLayout(hLayout[i]);
            vLayout[0]->addLayout(h_LbLayout[i]);
            vLayout[0]->addStretch();
        }
        else{
            vLayout[1]->addLayout(hLayout[i]);
            vLayout[1]->addLayout(h_LbLayout[i]);
            vLayout[1]->addStretch();
        }
    }
    QHBoxLayout * tmp_layout = new QHBoxLayout();
    QLabel *tmp_lbl = new QLabel(this);
    tmp_lbl->setText(tr("Select/Unselect all"));
    cbox_select_all = new QCheckBox(this);
    tmp_layout->addWidget(tmp_lbl);
    tmp_layout->addWidget(cbox_select_all);

    vLayout[0]->addLayout(tmp_layout);

    mainLayout = new QHBoxLayout();
    mainLayout->addLayout(vLayout[0]);
    //mainLayout->addStretch();
    mainLayout->addLayout(vLayout[1]);




    ui->centralWidget->setLayout(mainLayout);

    timer = new QTimer(this);
    timer->setInterval(50);
    /* udp socket setup*/
    fromMasterHand = new QUdpSocket(this);
    fromMasterHand->bind(QHostAddress::Any,FROM_MASTER_HAND_PORT);//slave hand transmit force data with this port
    connect(fromMasterHand,SIGNAL(readyRead()),this,SLOT(slave_hand_recv()));
    get_configuration();


    connect(timer,SIGNAL(timeout()),this,SLOT(timer_out()));
    timer->start();

    connect(cbox_select_all,SIGNAL(toggled(bool)),this,SLOT(select_all_checked(bool)));
    for(i = 0; i < 16; i++){
        joint_data.joint_force[i] = 0;
    }

    dat_log = new QFile("/home/zl/dat_log");
    dat_log->open(QIODevice::ReadWrite | QIODevice::Text);
    dat_log->seek(0);
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
    joint_state.name[0] = "joint-r1";
    joint_state.name[1] = "joint-r2";
    joint_state.name[2] = "joint-r3";
    joint_state.name[3] = "joint-r4";
    joint_state.name[4] = "joint-r5";
    joint_state.name[5] = "joint-r6";
    joint_state.name[6] = "joint-r7";
    joint_state.name[7] = "joint-l1";
    joint_state.name[8] = "joint-l2";
    joint_state.name[9] = "joint-l3";
    joint_state.name[10] = "joint-l4";
    joint_state.name[11] = "joint-l5";
    joint_state.name[12] = "joint-l6";
    joint_state.name[13] = "joint-l7";

    for(i = 0; i < 14; i++){
        int tmp = slider[i]->value();
        QString s;
        s.setNum(tmp);
        lineEdit[i]->setText(s);
        joint_state.position[i] = tmp / 100.0;

        s.setNum((float)(dh_angle[transfer_seq[i]]));
        le_dhangle[i]->setText(s);


    }

    //left
    if(cbox[0]->isChecked())
        joint_state.position[0] = dh_angle[8];
    if(cbox[1]->isChecked())
        joint_state.position[1] = dh_angle[9];
    if(cbox[2]->isChecked())
        joint_state.position[2] = dh_angle[10];
    if(cbox[3]->isChecked())
        joint_state.position[3] = dh_angle[11];
    if(cbox[4]->isChecked())
        joint_state.position[4] = dh_angle[12];
    if(cbox[5]->isChecked())
        joint_state.position[5] = dh_angle[13];
    if(cbox[6]->isChecked())
        joint_state.position[6] = dh_angle[14];
    //right
    if(cbox[7]->isChecked())
        joint_state.position[7] = dh_angle[0];
    if(cbox[8]->isChecked())
        joint_state.position[8] = dh_angle[1];
    if(cbox[9]->isChecked())
        joint_state.position[9] = dh_angle[2];
    if(cbox[10]->isChecked())
        joint_state.position[10] = dh_angle[3];
    if(cbox[11]->isChecked())
        joint_state.position[11] = dh_angle[4];
    if(cbox[12]->isChecked())
        joint_state.position[12] = dh_angle[5];
    if(cbox[13]->isChecked())
        joint_state.position[13] = dh_angle[6];


    jointstates_publisher.publish(joint_state);

    fromMasterHand->writeDatagram((char*)(&joint_data),sizeof(JOINT_DAT_TYPE),QHostAddress(master_addr),TO_MASTER_HAND_PORT);

}

//read id configuration from the config file
void j_state_pub::get_configuration(){
    int i;
    cfg = new QFile("/home/zl/paramlist.cfg");
    cfg->open(QIODevice::ReadWrite | QIODevice::Text);
    cfg->seek(0);
    QTextStream in(cfg);
    for(i = 0; i < 16 && !in.atEnd(); i++){
        QStringList list = in.readLine().split(',');
        cfg_UploadID_list[i] = list[1].toInt();
        cfg_Data_offset[i] = list[2].toInt();
        cfg_DownloadID_list[i] = list[3].toInt();
    }
    if(i != 16)
        qDebug()<<"bad configuration!";


}
#define J_ANG_1_MOD  1.8756   // used to allign center of enc1
//#define J_ANG_1_MOD  M_PI   // used to allign center of enc1
void j_state_pub::slave_hand_recv(){
    JOINT_DAT_TYPE recv_frame;
    int i;
    Matrix3d r1,r2,r3,BaseRot_L,BaseRot_R,Re_L,Re_R;
    QTextStream out(dat_log);
    static long int serial_nm = 0;
    BaseRot_L<<
                0, 1, 0,
                1, 0, 0,
                0, 0, -1;
    BaseRot_R<<
                0, 0, -1,
                0, -1, 0,
                -1, 0, 0;
    fromMasterHand->readDatagram((char*)(&joint_data),sizeof(JOINT_DAT_TYPE));
    for(i = 0; i < 16; i++){
        if(joint_data.joint_pos_raw[i] >= cfg_Data_offset[i])
            joint_data.joint_pos_abs[i] = joint_data.joint_pos_raw[i] - cfg_Data_offset[i];
        else
            joint_data.joint_pos_abs[i] = joint_data.joint_pos_raw[i] + enc_modula[i] - cfg_Data_offset[i];
        j_angle[i] = M_PI * joint_data.joint_pos_abs[i] / scale_factor[i] / 180.0 ;

        if(j_angle[i]> (M_PI))
            j_angle[i] = j_angle[i] - 2 * M_PI;
        //qDebug() << i<<"--"<< joint_data.joint_pos_abs[i]<<endl;
        //qDebug() << i<<"--"<< j_angle[i]<<endl;
    }
    double cos_alpha = qCos(qDegreesToRadians(35.0));
    double sin_alpha = qSin(qDegreesToRadians(35.0));

    QString s;

    double theta_r1 = j_angle[0] - j_angle[1]  / 2.0 ;
	double theta_r2 = 4.0 * qAcos( cos_alpha / qCos(qAcos(cos_alpha * cos_alpha + sin_alpha * sin_alpha * qCos(j_angle[1] + J_ANG_1_MOD)) / 2.0)) - M_PI_2;
	double theta_r3 = j_angle[2] + j_angle[1] / 2.0 ;

	s.setNum((float)(theta_r1));
	le_jangle[7]->setText(s);
	s.setNum((float)(theta_r2));
	le_jangle[8]->setText(s);
	s.setNum((float)(theta_r3));
	le_jangle[9]->setText(s);

    double theta_cr = atan2(tan(theta_r2 ) , cos(theta_r1));
    AngleAxisd rotr1(-theta_r1, Vector3d(0, 1, 0));
    AngleAxisd rotr2(theta_cr, Vector3d(1, 0, 0));
    AngleAxisd rotr3(-theta_r3, Vector3d(0, 0, 1));
    Re_R = rotr1.matrix() * BaseRot_R * rotr2.matrix() * rotr3.matrix();
    double *MatptrR = Re_R.data();

    //std::cout<< "Rotation1"<<std::endl<< rot1.matrix() <<std::endl;
    //std::cout<<"cos_t1="<< cos(theta_r1) <<"cos_t2="<< cos(theta_r2) <<"theta_cr="<< theta_cr << "Rotation2"<<std::endl<< rot2.matrix() <<std::endl;
    //std::cout<< "Rotation3"<<std::endl<< rot3.matrix() <<std::endl;

    dh_angle[0] = -atan(MatptrR[1] / MatptrR[2]);
    dh_angle[1] = asin(MatptrR[0]);
    dh_angle[2] = atan(MatptrR[3] / MatptrR[6]);


    //dh_angle[0] = j_angle[0] - j_angle[1]  / 2.0 ;
    //dh_angle[1] = 4.0 * qAcos( cos_alpha / qCos(qAcos(cos_alpha * cos_alpha + sin_alpha * sin_alpha * qCos(j_angle[1] + J_ANG_1_MOD)) / 2.0)) - M_PI_2;
    //dh_angle[2] = j_angle[2] + j_angle[1] / 2.0 ;
    dh_angle[3] = -j_angle[3] - M_PI_2;
    dh_angle[4] = -j_angle[4];
    dh_angle[5] = j_angle[5] ;
    dh_angle[6] = j_angle[6] ;

    theta_r1 = -j_angle[8] + j_angle[9]  / 2.0 ;
    theta_r2 = -4.0 * qAcos( cos_alpha / qCos(qAcos(cos_alpha * cos_alpha + sin_alpha * sin_alpha * qCos(j_angle[9] + J_ANG_1_MOD)) / 2.0)) + M_PI_2;
    theta_r3 = j_angle[10] - j_angle[9] / 2.0;

    s.setNum((float)(theta_r1));
    le_jangle[0]->setText(s);
    s.setNum((float)(theta_r2));
    le_jangle[1]->setText(s);
    s.setNum((float)(theta_r3));
    le_jangle[2]->setText(s);


    theta_cr = atan2(tan(theta_r2 ) , cos(theta_r1));
    AngleAxisd rotl1(-theta_r1, Vector3d(0, 1, 0));
    AngleAxisd rotl2(theta_cr, Vector3d(0, 0, 1));
    AngleAxisd rotl3(-theta_r3, Vector3d(0, 1, 0));
    Re_L = rotl1.matrix() * BaseRot_L * rotl2.matrix() * rotl3.matrix();
    double *Matptr = Re_L.data();

    std::cout<< "Rotation1"<<std::endl<< rotl1.matrix() <<std::endl;
    std::cout<<"cos_t1="<< cos(theta_r1) <<"cos_t2="<< cos(theta_r2) <<"theta_cr="<< theta_cr << "Rotation2"<<std::endl<< rotl2.matrix() <<std::endl;
    std::cout<< "Rotation3"<<std::endl<< rotl3.matrix() <<std::endl;


    dh_angle[8] = -atan(-Matptr[7] / Matptr[8]);
    dh_angle[9] = asin(Matptr[6]);
    dh_angle[10] = atan2(Matptr[0],Matptr[3]);

/*    out << ++serial_nm << ","
        << theta_r1 << ","
        << theta_r2 << ","
        << theta_r3 << ","
        << dh_angle[8] << ","
        << dh_angle[9] << ","
        << dh_angle[10] << ","
        <<endl;
    out.flush();
    */
    dh_angle[11] = j_angle[11] + M_PI_2;
    dh_angle[12] = -j_angle[12];
    dh_angle[13] = j_angle[13] ;
    dh_angle[14] = j_angle[14] ;


    for(i = 0; i < 16; i++){
        qDebug() << i<<"--"<< j_angle[i] << "--" <<dh_angle[i]<<endl;
    }
    // qDebug() << recv_frame.joint_force;
   /*

    for(i = 0; i < 7; i++){
        qDebug() << i<<"--"<< joint_data.joint_pos_abs[i]<<endl;

    }
*/
}

void j_state_pub::select_all_checked(bool checked){
    int i;
    if(checked){
        for(i = 0; i<15; i++){
            cbox[i]->setChecked(true);
        }
    }
    else{
        for(i = 0; i<15; i++){
            cbox[i]->setChecked(false);
        }

    }

}
