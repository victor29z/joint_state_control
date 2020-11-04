#include "j_state_pub.h"
#include "ui_j_state_pub.h"
#include <QtMath>
#include <iostream>
#include<QDateTime>
#define FROM_MASTER_HAND_PORT  12625
#define TO_MASTER_HAND_PORT 17362
#define TO_SLAVE_PORT 9191

using namespace Eigen;
char master_hand_addr[20] = "";
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

const char* j_name_list[]={
    "bottom_joint1",
    "bottom_joint2",
    "shoulder2_joint1",
    "elbow1_joint1",
    "elbow2_joint1",
    "artifice1_joint1",
    "artifice2_joint1",
    "artifice3_Joint1",
    "hand1_joint1",
    "fingure1_joint1",
    "fingure2_joint1",
    "shoulder2_joint2",
    "elbow1_joint2",
    "elbow2_joint2",
    "artifice1_joint2",
    "artifice2_joint2",
    "artifice3_joint2",
    "hand2_joint2",
    "fingure1_joint2",
    "fingure2_joint2"
};

const int transfer_seq[14] = {0,1,2,3,4,5,6, 8,9,10,11,12,13,14};

j_state_pub::j_state_pub(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::j_state_pub)
{
    int i;
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
    setup_ui();
    //build and initialize joint state publisher
	jointstates_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
    Joint_State_Msg_Initialize(NO_OF_JOINTS,(char**)j_name_list);



    timer = new QTimer(this);
    timer->setInterval(50);
    /* udp socket setup*/
    fromMasterHand = new QUdpSocket(this);
    fromMasterHand->bind(QHostAddress::Any,FROM_MASTER_HAND_PORT);//slave hand transmit force data with this port

    toSlaveSock = new QUdpSocket(this);
    connect(fromMasterHand,SIGNAL(readyRead()),this,SLOT(slave_hand_recv()));
    get_configuration();


    connect(timer,SIGNAL(timeout()),this,SLOT(timer_out()));
    timer->start();

    connect(cbox_select_all,SIGNAL(toggled(bool)),this,SLOT(select_all_checked(bool)));
    for(i = 0; i < 16; i++){
        joint_data.joint_force[i] = 0;
    }

    dat_log = new QFile("/home/robot/dat_log");
    dat_log->open(QIODevice::ReadWrite | QIODevice::Text);
    dat_log->seek(0);

    waist_lr = 0;
    waist_ud = 0;
    joint_data.keyvalue[7] = 0;
    joint_data.keyvalue[15] = 0;
    handle_data.right.joystick_lr = 0x850;
    handle_data.right.joystick_ud = 0x850;
    //select_all_checked(true);
    cbox_select_all->setChecked(true);

}

j_state_pub::~j_state_pub()
{
    delete ui;
}

void j_state_pub::timer_out(void){
    int i;
    QString s;
	joint_state.header.stamp = ros::Time::now();
    for(i = 0; i < NO_OF_JOINTS; i++){
        int tmp = slider[i]->value();

        s.setNum(tmp);
        lineEdit[i]->setText(s);//show the slider position
        joint_state.position[i] = tmp / 100.0;
    }

    //right
    if(handle_data.right.enable_key){
        if(cbox[0]->isChecked())
        {joint_state.position[2] = dh_angle[0];s.setNum((float)(dh_angle[0]));le_dhangle[2]->setText(s);}//show the actual command data that was sent to slave joint
        if(cbox[1]->isChecked())
        {joint_state.position[3] = dh_angle[1];s.setNum((float)(dh_angle[1]));le_dhangle[3]->setText(s);}
        if(cbox[2]->isChecked())
        {joint_state.position[4] = dh_angle[2];s.setNum((float)(dh_angle[2]));le_dhangle[4]->setText(s);}
        if(cbox[3]->isChecked())
        {joint_state.position[5] = dh_angle[3];s.setNum((float)(dh_angle[3]));le_dhangle[5]->setText(s);}
        if(cbox[4]->isChecked())
        {joint_state.position[6] = dh_angle[4];s.setNum((float)(dh_angle[4]));le_dhangle[6]->setText(s);}
        if(cbox[5]->isChecked())
        {joint_state.position[7] = dh_angle[5];s.setNum((float)(dh_angle[5]));le_dhangle[7]->setText(s);}
        if(cbox[6]->isChecked())
        {joint_state.position[8] = dh_angle[6];s.setNum((float)(dh_angle[6]));le_dhangle[8]->setText(s);}
        //hand control
        if(handle_data.right.buttom_key)
            {joint_state.position[9] = -0.5;joint_state.position[10] = 0.5;}



    }
    // use right handle joystick to control waist joint
   if(cbox_select_all->isChecked()){
       if(handle_data.right.joystick_lr < 0x600)
           if(waist_lr < 1.57)waist_lr+=0.02;
       if(handle_data.right.joystick_lr > 0xa00)
           if(waist_lr > -1.57)waist_lr-=0.02;
       joint_state.position[0] = waist_lr;

       if(handle_data.right.joystick_ud < 0x600)
           if(waist_ud < 1.57)waist_ud-=0.02;
       if(handle_data.right.joystick_ud > 0xa00)
           if(waist_ud > -1.57)waist_ud+=0.02;
       joint_state.position[1] = waist_ud;


   }

    //left
    if(handle_data.left.enable_key){
        if(cbox[7]->isChecked())
        {joint_state.position[11] = dh_angle[8];s.setNum((float)(dh_angle[8]));le_dhangle[11]->setText(s);}
        if(cbox[8]->isChecked())
        {joint_state.position[12] = dh_angle[9];s.setNum((float)(dh_angle[9]));le_dhangle[12]->setText(s);}
        if(cbox[9]->isChecked())
        {joint_state.position[13] = dh_angle[10];s.setNum((float)(dh_angle[10]));le_dhangle[13]->setText(s);}
        if(cbox[10]->isChecked())
        {joint_state.position[14] = dh_angle[11];s.setNum((float)(dh_angle[11]));le_dhangle[14]->setText(s);}
        if(cbox[11]->isChecked())
        {joint_state.position[15] = dh_angle[12];s.setNum((float)(dh_angle[12]));le_dhangle[15]->setText(s);}
        if(cbox[12]->isChecked())
        {joint_state.position[16] = dh_angle[13];s.setNum((float)(dh_angle[13]));le_dhangle[16]->setText(s);}
        if(cbox[13]->isChecked())
        {joint_state.position[17] = dh_angle[14];s.setNum((float)(dh_angle[14]));le_dhangle[17]->setText(s);}
        //hand control
        if(handle_data.left.buttom_key)
            {joint_state.position[18] = -0.5;joint_state.position[19] = -0.5;}

    }



    jointstates_publisher.publish(joint_state);
    /*
    if(joint_data.keyvalue[7]) dh_angle[7] = 1.0;
    else dh_angle[7] = 0;
*/
    fromMasterHand->writeDatagram((char*)(&joint_data),sizeof(JOINT_DAT_TYPE),QHostAddress(master_addr),TO_MASTER_HAND_PORT);
    toSlaveSock->writeDatagram((char*)(&dh_angle),sizeof(dh_angle),QHostAddress("127.0.0.1"),TO_SLAVE_PORT);

}

//read id configuration from the config file
void j_state_pub::get_configuration(){
    int i;
    cfg = new QFile("/home/robot/paramlist.cfg");
    //cfg = new QFile("./paramlist.cfg");
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
    QTextStream log_out(dat_log);
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
    //read handle value
    handle_data.left.top_key1 = joint_data.keyvalue[15] & KEY_MASK_TOPKEY1;
    handle_data.left.top_key2 = joint_data.keyvalue[15] & KEY_MASK_TOPKEY2;
    handle_data.left.top_key3 = joint_data.keyvalue[15] & KEY_MASK_TOPKEY3;
    handle_data.left.buttom_key = joint_data.keyvalue[15] & KEY_MASK_BUTKEY;
    handle_data.left.enable_key = joint_data.keyvalue[15] & KEY_MASK_ENABLEKEY;
    handle_data.left.joystick_lr = joint_data.joint_pos_raw[15] & JOYSTICK_LR_MUSK;
    handle_data.left.joystick_ud = (joint_data.joint_pos_raw[15] & JOYSTICK_UD_MUSK) >> 12;

    handle_data.right.top_key3 = joint_data.keyvalue[7] & KEY_MASK_TOPKEY1;
    handle_data.right.top_key2 = joint_data.keyvalue[7] & KEY_MASK_TOPKEY2;
    handle_data.right.top_key1 = joint_data.keyvalue[7] & KEY_MASK_TOPKEY3;
    handle_data.right.buttom_key = joint_data.keyvalue[7] & KEY_MASK_BUTKEY;
    handle_data.right.enable_key = joint_data.keyvalue[7] & KEY_MASK_ENABLEKEY;
    handle_data.right.joystick_lr = joint_data.joint_pos_raw[7] & JOYSTICK_LR_MUSK;
    handle_data.right.joystick_ud = (joint_data.joint_pos_raw[7] & JOYSTICK_UD_MUSK) >> 12;
    double cos_alpha = qCos(qDegreesToRadians(35.0));
    double sin_alpha = qSin(qDegreesToRadians(35.0));

    QString s;

    double theta_r1 = j_angle[0] - j_angle[1]  / 2.0 ;
	double theta_r2 = 4.0 * qAcos( cos_alpha / qCos(qAcos(cos_alpha * cos_alpha + sin_alpha * sin_alpha * qCos(j_angle[1] + J_ANG_1_MOD)) / 2.0)) - M_PI_2;
	double theta_r3 = j_angle[2] + j_angle[1] / 2.0 ;

    s.setNum((float)(theta_r1));//show the caculated shoulder joint data
    le_jangle[2]->setText(s);
	s.setNum((float)(theta_r2));
    le_jangle[3]->setText(s);
	s.setNum((float)(theta_r3));
    le_jangle[4]->setText(s);

    double theta_cr = atan2(tan(theta_r2 ) , cos(theta_r1));
    AngleAxisd rotr1(-theta_r1, Vector3d(0, 1, 0));
    //AngleAxisd rotr2(theta_cr, Vector3d(1, 0, 0));
    AngleAxisd rotr2(theta_r2, Vector3d(1, 0, 0));
    AngleAxisd rotr3(-theta_r3, Vector3d(0, 0, 1));
    Re_R = rotr1.matrix() * BaseRot_R * rotr2.matrix() * rotr3.matrix();
    double *MatptrR = Re_R.data();

    //std::cout<< "Rotation1"<<std::endl<< rot1.matrix() <<std::endl;
    //std::cout<<"cos_t1="<< cos(theta_r1) <<"cos_t2="<< cos(theta_r2) <<"theta_cr="<< theta_cr << "Rotation2"<<std::endl<< rot2.matrix() <<std::endl;
    //std::cout<< "Rotation3"<<std::endl<< rot3.matrix() <<std::endl;

    dh_angle[0] = -atan2(MatptrR[1] , MatptrR[2]) -M_PI;
    if(dh_angle[0] > M_PI) dh_angle[0] -= M_PI*2;
    if(dh_angle[0] < -M_PI) dh_angle[0] += M_PI*2;


    dh_angle[1] = asin(MatptrR[0]);
    dh_angle[2] = -atan2(MatptrR[3] , MatptrR[6]) -M_PI;
    if(dh_angle[2] > M_PI) dh_angle[2] -= M_PI*2;
    if(dh_angle[2] < -M_PI) dh_angle[2] += M_PI*2;

//qDebug() <<"Mat(3)"<<MatptrR[3]<<",Mat(6)"<<MatptrR[6]<<endl;

    //dh_angle[0] = j_angle[0] - j_angle[1]  / 2.0 ;
    //dh_angle[1] = 4.0 * qAcos( cos_alpha / qCos(qAcos(cos_alpha * cos_alpha + sin_alpha * sin_alpha * qCos(j_angle[1] + J_ANG_1_MOD)) / 2.0)) - M_PI_2;
    //dh_angle[2] = j_angle[2] + j_angle[1] / 2.0 ;
    dh_angle[3] = j_angle[3] + M_PI_2;
    dh_angle[4] = -j_angle[4];
    dh_angle[5] = -j_angle[5] ;
    dh_angle[6] = -j_angle[6] ;

    theta_r1 = -j_angle[8] + j_angle[9]  / 2.0 ;
    theta_r2 = -4.0 * qAcos( cos_alpha / qCos(qAcos(cos_alpha * cos_alpha + sin_alpha * sin_alpha * qCos(j_angle[9] + J_ANG_1_MOD)) / 2.0)) + M_PI_2;
    theta_r3 = j_angle[10] - j_angle[9] / 2.0;

    s.setNum((float)(theta_r1));
    le_jangle[11]->setText(s);
    s.setNum((float)(theta_r2));
    le_jangle[12]->setText(s);
    s.setNum((float)(theta_r3));
    le_jangle[13]->setText(s);


    theta_cr = atan2(tan(theta_r2 ) , cos(theta_r1));
    AngleAxisd rotl1(-theta_r1, Vector3d(0, 1, 0));
    //AngleAxisd rotl2(theta_cr, Vector3d(0, 0, 1));
    AngleAxisd rotl2(theta_r2, Vector3d(0, 0, 1));
    AngleAxisd rotl3(-theta_r3, Vector3d(0, 1, 0));
    Re_L = rotl1.matrix() * BaseRot_L * rotl2.matrix() * rotl3.matrix();
    double *Matptr = Re_L.data();

    //std::cout<< "Rotation1"<<std::endl<< rotl1.matrix() <<std::endl;
    //std::cout<<"cos_t1="<< cos(theta_r1) <<"cos_t2="<< cos(theta_r2) <<"theta_cr="<< theta_cr << "Rotation2"<<std::endl<< rotl2.matrix() <<std::endl;
    //std::cout<< "Rotation3"<<std::endl<< rotl3.matrix() <<std::endl;
//std::cout<<"theta_r2="<< theta_r2 <<std::endl<<"theta_cr="<< theta_cr<<std::endl;

    dh_angle[8] = atan2(-Matptr[7] , Matptr[8]) - M_PI;
    if(dh_angle[8] > M_PI) dh_angle[8] -= M_PI*2;
    if(dh_angle[8] < -M_PI) dh_angle[8] += M_PI*2;

    //qDebug() <<"Mat(7)"<<Matptr[7]<<",Mat(8)"<<Matptr[8]<<endl;
    dh_angle[9] = asin(Matptr[6]);
    dh_angle[10] = -atan2(Matptr[0],Matptr[3]);
    if(dh_angle[10] > M_PI) dh_angle[10] -= M_PI*2;
    if(dh_angle[10] < -M_PI) dh_angle[10] += M_PI*2;



    dh_angle[11] = j_angle[11] + M_PI_2;
    dh_angle[12] = -j_angle[12];
    dh_angle[13] = j_angle[13] ;
    dh_angle[14] = -j_angle[14] ;
/*print debug infromation*/
    for(i = 0; i < 16; i++){
        qDebug() << i<<"--"<< j_angle[i] << "--" <<dh_angle[i]<<endl;

    }

    if(!handle_data.right.enable_key) return;
    /*output debug log*/
    QDateTime dateTime = QDateTime::currentDateTime();
    // 字符串格式化
    QString timestamp = dateTime.toString("yyyy-MM-dd hh:mm:ss.zzz");
    // 获取毫秒值
    int ms = dateTime.time().msec();
    // 转换成时间戳
    qint64 epochTime = dateTime.toMSecsSinceEpoch();
    log_out <<serial_nm++<<","<< epochTime << ",";
    for(i = 0; i < 16; i++){
        qDebug() << i<<"--"<< j_angle[i] << "--" <<dh_angle[i]<<endl;
        log_out << dh_angle[i] << ",";

    }
    log_out << endl;
    log_out.flush();
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
        for(i = 0; i<NO_OF_JOINTS; i++){
            cbox[i]->setChecked(true);
        }
    }
    else{
        for(i = 0; i<NO_OF_JOINTS; i++){
            cbox[i]->setChecked(false);
        }

    }

}

void j_state_pub::Joint_State_Msg_Initialize(int size, char* joint_name_list[]){
    int i;
    joint_state.name.resize(size);
    joint_state.position.resize(size);
    for(i = 0; i < size; i++)
        joint_state.name[i] = joint_name_list[i];

}

void j_state_pub::setup_ui(){
    vLayout[0] = new QVBoxLayout();
    vLayout[1] = new QVBoxLayout();
    vLayout[2] = new QVBoxLayout();
    vLayout[3] = new QVBoxLayout();
    int i;
    for(i = 0; i < NO_OF_JOINTS; i++){
        hLayout[i] = new QHBoxLayout();
        h_LbLayout[i] = new QHBoxLayout();
        lbl[i] = new QLabel(this);
        le_jangle[i] = new QLineEdit(this);
        le_dhangle[i] = new QLineEdit(this);
        slider[i] = new QSlider(this);
        lineEdit[i] = new QLineEdit(this);
        cbox[i] = new QCheckBox(this);

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
    }
    for(i = 2; i < NO_OF_JOINTS; i++){
        QString s;

        if(i < (DOF_EACH_ARM+2)){
            s.setNum(i-2);
            lbl[i]->setText(tr("R")+s);
        }
        else{
            s.setNum(i-11);
            lbl[i]->setText(tr("L")+s);
        }


        if(i < (DOF_EACH_ARM+2)){
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
    lbl[0]->setText(tr("Body1"));
    lbl[1]->setText(tr("Body2"));
    QHBoxLayout * tmp_layout = new QHBoxLayout();
    QLabel *tmp_lbl = new QLabel(this);
    tmp_lbl->setText(tr("Select/Unselect all"));
    cbox_select_all = new QCheckBox(this);
    tmp_layout->addWidget(tmp_lbl);
    tmp_layout->addWidget(cbox_select_all);

    vLayout[2]->addLayout(hLayout[0]);
    vLayout[2]->addStretch();
    vLayout[2]->addLayout(hLayout[1]);
    vLayout[2]->addStretch();
    vLayout[3]->addLayout(tmp_layout);

    mainLayout = new QVBoxLayout();
    arm_asm_Layout = new QHBoxLayout();
    arm_asm_Layout->addLayout(vLayout[1]);
    //mainLayout->addStretch();
    arm_asm_Layout->addLayout(vLayout[0]);
    mainLayout->addLayout(vLayout[2]);
    mainLayout->addLayout(arm_asm_Layout);
    mainLayout->addLayout(vLayout[3]);




    ui->centralWidget->setLayout(mainLayout);

}
