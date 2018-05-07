#include "arm_gui.h"
#include "ui_arm_gui.h"
#include <QTextStream>
#include <QPixmap>
#include <QIcon>

#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <chrono>

Arm_gui::Arm_gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Arm_gui)
    {
    
    ui->setupUi(this);
    ui->comboBox->addItem("No Backend");
    ui->comboBox->addItem("Feetech");
    ui->comboBox->addItem("Arduino");
    ui->comboBox_2->addItem("Using Right Arm");
    ui->comboBox_2->addItem("Using Left Arm");
    ui->checkBox->setChecked(false);

    connect(ui->Open_Claw, SIGNAL(clicked()), this, SLOT(Open_ClawClicked()));
    connect(ui->Close_Claw, SIGNAL(clicked()), this, SLOT(Close_ClawClicked()));
    connect(ui->Home, SIGNAL(clicked()), this, SLOT(HomeClicked()));
    connect(ui->Run_autopose, SIGNAL(clicked()), this, SLOT(Run_autoposeClicked()));
    connect(ui->Run_joints, SIGNAL(clicked()), this, SLOT(Run_jointsClicked()));
    connect(ui->Run_position, SIGNAL(clicked()), this, SLOT(Run_positionClicked()));
    connect(ui->Stop_Claw, SIGNAL(clicked()), this, SLOT(Stop_ClawClicked()));
    connect(ui->checkBox, SIGNAL(clicked(bool)), this, SLOT(checkBoxClicked(bool)));

    connect(ui->ConnectB, SIGNAL(clicked()), this, SLOT(ConnectBClicked()));
    connect(ui->ConnectA, SIGNAL(clicked()), this, SLOT(ConnectAClicked()));

    connect(ui->X1, SIGNAL(clicked()), this, SLOT(X1Clicked()));
    connect(ui->X2, SIGNAL(clicked()), this, SLOT(X2Clicked()));
    connect(ui->Y1, SIGNAL(clicked()), this, SLOT(Y1Clicked()));
    connect(ui->Y2, SIGNAL(clicked()), this, SLOT(Y2Clicked()));
    connect(ui->Z1, SIGNAL(clicked()), this, SLOT(Z1Clicked()));
    connect(ui->Z2, SIGNAL(clicked()), this, SLOT(Z2Clicked()));

    connect(ui->Run_readLoadPos1, SIGNAL(clicked()), this, SLOT(Run_readPosLoad1Clicked()));
    connect(ui->Run_readLoadPosC, SIGNAL(clicked()), this, SLOT(Run_readPosLoadCClicked()));
    connect(ui->Stop_readLoadPosC, SIGNAL(clicked()), this, SLOT(Stop_readPosLoadCClicked()));
    

    connect(ui->Run_addwaypoint, SIGNAL(clicked()), this, SLOT(Run_addWayPointClicked()));
    connect(ui->Run_waypoints, SIGNAL(clicked()), this, SLOT(Run_WayPointsClicked()));
    
    QPixmap pixmapX1("src/vigus_gui/general_gui/images/arrow_up.png");
    QIcon ButtonIconX1(pixmapX1);
    ui->X1->setIcon(ButtonIconX1);
    ui->X1->setIconSize(pixmapX1.rect().size());

    QPixmap pixmapX2("src/vigus_gui/general_gui/images/arrow_down.png");
    QIcon ButtonIconX2(pixmapX2);
    ui->X2->setIcon(ButtonIconX2);
    ui->X2->setIconSize(pixmapX2.rect().size());

    QPixmap pixmapY1("src/vigus_gui/general_gui/images/arrow_left.png");
    QIcon ButtonIconY1(pixmapY1);
    ui->Y1->setIcon(ButtonIconY1);
    ui->Y1->setIconSize(pixmapY1.rect().size());

    QPixmap pixmapY2("src/vigus_gui/general_gui/images/arrow_right.png");
    QIcon ButtonIconY2(pixmapY2);
    ui->Y2->setIcon(ButtonIconY2);
    ui->Y2->setIconSize(pixmapY2.rect().size());

    QPixmap pixmapZ1("src/vigus_gui/general_gui/images/arrow_up.png");
    QIcon ButtonIconZ1(pixmapZ1);
    ui->Z1->setIcon(ButtonIconZ1);
    ui->Z1->setIconSize(pixmapZ1.rect().size());

    QPixmap pixmapZ2("src/vigus_gui/general_gui/images/arrow_down.png");
    QIcon ButtonIconZ2(pixmapZ2);
    ui->Z2->setIcon(ButtonIconZ2);
    ui->Z2->setIconSize(pixmapZ2.rect().size());

    ui->scrollArea_6->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    ui->scrollArea_6->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);

    }

//---------------------------------------------------------------------------------------------------------------------
Arm_gui::~Arm_gui()
{
    delete ui;
}

//---------------------------------------------------------------------------------------------------------------------
bool Arm_gui::configureGUI(std::vector<std::pair<std::string, std::string>> _config){

    for( int i = 0; i < _config.size(); i++){
        if( _config[i].first == "idArm"){
            mIdArm = _config[i].second;
            ui->lineEdit_ID->setText(QString::fromStdString(mIdArm));
        }        
        if( _config[i].first == "SerialPort"){
            mSerialPortArm = _config[i].second;
            ui->lineEdit_serial->setText(QString::fromStdString(mSerialPortArm));
        }
        if( _config[i].first == "Visualizer"){
            if(_config[i].second == "true"){
                mVisualizer = true;
            }else{
                mVisualizer = false;
            }
        } 
        if( _config[i].first == "RobotFile"){
            mRobotFile = _config[i].second;
        }
        if( _config[i].first == "EnviromentFile"){
            mEnviromentFile = _config[i].second;
        }
        if( _config[i].first == "Backend"){
            mBackendArm = _config[i].second;
            if(mBackendArm == "Feetech"){
                 ui->comboBox->setCurrentIndex(1);
            }else if(mBackendArm == "Arduino"){
                ui->comboBox->setCurrentIndex(2);
            }else{
                ui->comboBox->setCurrentIndex(0);
            }
        }

    }
    
    //changeBackend(mBackendArm);
    return true;

}
//---------------------------------------------------------------------------------------------------------------------
bool Arm_gui::changeBackend(std::string _backend){

    if(_backend == "Gazebo"){

        backendConfig1.type = hecatonquiros::Backend::Config::eType::Gazebo;
        backendConfig1.topic = "left_arm/joint_states";
        backendConfig1.armId =1;
        backendConfig2.type = hecatonquiros::Backend::Config::eType::Gazebo;
        backendConfig2.topic = "right_arm/joint_states";
        backendConfig2.armId =2;

    }else if(_backend == "Arduino"){

        QString qSerialPort;
        qSerialPort = ui->lineEdit_serial->text();
        mSerialPortArm = qSerialPort.toStdString();
		serial::Serial* arduinoCom = new serial::Serial(mSerialPortArm, 115200, serial::Timeout::simpleTimeout(1000));
		if (!arduinoCom->isOpen()) {
			std::cout << "Could not open serial port" << std::endl;
            return false;
		}

		backendConfig1.type = hecatonquiros::Backend::Config::eType::Arduino; backendConfig1.sharedSerialPort = arduinoCom; backendConfig1.armId =1;
		backendConfig2.type = hecatonquiros::Backend::Config::eType::Arduino; backendConfig2.sharedSerialPort = arduinoCom; backendConfig2.armId =2;
    
    }else if(_backend == "Feetech"){

        QString qSerialPort;
        qSerialPort = ui->lineEdit_serial->text();
        mSerialPortArm = qSerialPort.toStdString();

		//backendConfig1.valuesMinMax = { {-109.376, 106.492},  
        //                            	{-111.490, 107.986},
        //                            	{-104.869, 113.775},
        //                            	{-111.568, 98.624},
        //                            	{-152.400, 144.563},
        //                            	{-135.305, 157.744}};
        //backendConfig1.jointsOffsets = { 0,	0,	0, 0,	0,	0 };

		//backendConfig2.valuesMinMax = { {-112.737, 100.442},  
        //                                {-106.202, 110.798},
        //                                {-103.401, 116.820},
        //                                {-92.791, 120.046},
        //                                {-152.098, 136.935},
        //                                {-147.468, 147.564}};
        //backendConfig2.jointsOffsets = { 0,	0,	0, 0,	0,	0};

        backendConfig1.configXML = "src/hecatonquiros/arm_controller/config/config_arm1.xml";
		backendConfig2.configXML = "src/hecatonquiros/arm_controller/config/config_arm2.xml";

		backendConfig1.type = hecatonquiros::Backend::Config::eType::Feetech; backendConfig1.port = mSerialPortArm; backendConfig1.armId =1;
		backendConfig2.type = hecatonquiros::Backend::Config::eType::Feetech; backendConfig2.port = mSerialPortArm; backendConfig2.armId =2;
		
    }else if(_backend == "No Backend"){
        backendConfig1.type = hecatonquiros::Backend::Config::eType::Dummy;
        backendConfig2.type = hecatonquiros::Backend::Config::eType::Dummy;
    }else{
        std::cout << "unrecognized mode, exiting" << std::endl;
        return false;
    }

    hecatonquiros::ModelSolver::Config modelSolverConfig1;
    modelSolverConfig1.type = hecatonquiros::ModelSolver::Config::eType::OpenRave;
    modelSolverConfig1.robotName = "left_arm";
    modelSolverConfig1.manipulatorName = "manipulator";
    if(mRobotFile != "" && mEnviromentFile == ""){
        modelSolverConfig1.robotFile = mRobotFile;
    }else if(mRobotFile == "" && mEnviromentFile != ""){
        modelSolverConfig1.environment = mEnviromentFile;
    }else if(mRobotFile == "" && mEnviromentFile == ""){
        std::cout << "ERROR! NO has puesto RobotFile o EnviromentFile" << std::endl;
        return false;
    }else{
        std::cout << "ERROR! Has puesto RobotFile y EnviromentFile" << std::endl;
        return false;
    }
    modelSolverConfig1.offset = {0.20,0.14,-0.04};
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(0*M_PI, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(1*M_PI,  Eigen::Vector3f::UnitY());
	Eigen::Quaternionf q(m);
    modelSolverConfig1.rotation = {q.w(), q.x(), q.y(), q.z()};
    if(mVisualizer == true){
        modelSolverConfig1.visualizer = true;
    }else{
        modelSolverConfig1.visualizer = false;
    }
    
    hecatonquiros::ModelSolver::Config modelSolverConfig2;
    modelSolverConfig2.type = hecatonquiros::ModelSolver::Config::eType::OpenRave;
    modelSolverConfig2.robotName = "right_arm";
    modelSolverConfig2.manipulatorName = "manipulator";
    if(mRobotFile != ""){
        modelSolverConfig2.robotFile = mRobotFile;
    }
    modelSolverConfig2.offset = {0.2,-0.2,-0.04};
    modelSolverConfig2.rotation = {q.w(), q.x(), q.y(), q.z()};
    if(mVisualizer == true){
        modelSolverConfig2.visualizer = true;
    }else{
        modelSolverConfig2.visualizer = false;
    }

    leftArm = new hecatonquiros::Arm4DoF (modelSolverConfig1, backendConfig1);
    rightArm = new hecatonquiros::Arm4DoF (modelSolverConfig2, backendConfig2);

    leftArm->home();
    rightArm->home();

    //armInUse = rightArm;
    //mUsingRight = true;
    //std::cout << "USING RIGHT ARM, start: " << std::endl;

    return true;

}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::X1Clicked(){
    QString qPrecision;
    qPrecision = ui->lineEdit_precision->text();
    float fPrecision;
    fPrecision = qPrecision.toFloat();

    auto pose = armInUse->pose();
    pose(0,3) = pose(0,3) + fPrecision;

    hecatonquiros::ModelSolver::IK_TYPE type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D; 

    std::vector<float> joints;
	if(armInUse->checkIk(pose, joints, type)){
		armInUse->joints(joints, true);
	}else{
		std::cout << "Not found IK" << std::endl;
	}

}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::X2Clicked(){
    QString qPrecision;
    qPrecision = ui->lineEdit_precision->text();
    float fPrecision;
    fPrecision = qPrecision.toFloat();

    auto pose = armInUse->pose();
    pose(0,3) = pose(0,3) - fPrecision;

    hecatonquiros::ModelSolver::IK_TYPE type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D; 

    std::vector<float> joints;
	if(armInUse->checkIk(pose, joints, type)){
		armInUse->joints(joints, true);
	}else{
		std::cout << "Not found IK" << std::endl;
	}

}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::Y1Clicked(){
    QString qPrecision;
    qPrecision = ui->lineEdit_precision->text();
    float fPrecision;
    fPrecision = qPrecision.toFloat();

    auto pose = armInUse->pose();
    pose(1,3) = pose(1,3) + fPrecision;

    hecatonquiros::ModelSolver::IK_TYPE type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D; 

    std::vector<float> joints;
	if(armInUse->checkIk(pose, joints, type)){
		armInUse->joints(joints, true);
	}else{
		std::cout << "Not found IK" << std::endl;
	}

}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::Y2Clicked(){
    QString qPrecision;
    qPrecision = ui->lineEdit_precision->text();
    float fPrecision;
    fPrecision = qPrecision.toFloat();

    auto pose = armInUse->pose();
    pose(1,3) = pose(1,3) - fPrecision;

    hecatonquiros::ModelSolver::IK_TYPE type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D; 

    std::vector<float> joints;
	if(armInUse->checkIk(pose, joints, type)){
		armInUse->joints(joints, true);
	}else{
		std::cout << "Not found IK" << std::endl;
	}

}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::Z1Clicked(){
    QString qPrecision;
    qPrecision = ui->lineEdit_precision->text();
    float fPrecision;
    fPrecision = qPrecision.toFloat();

    auto pose = armInUse->pose();
    pose(2,3) = pose(2,3) + fPrecision;

    hecatonquiros::ModelSolver::IK_TYPE type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D; 

    std::vector<float> joints;
	if(armInUse->checkIk(pose, joints, type)){
		armInUse->joints(joints, true);
	}else{
		std::cout << "Not found IK" << std::endl;
	}

}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::Z2Clicked(){
    QString qPrecision;
    qPrecision = ui->lineEdit_precision->text();
    float fPrecision;
    fPrecision = qPrecision.toFloat();

    auto pose = armInUse->pose();
    pose(2,3) = pose(2,3) - fPrecision;

    hecatonquiros::ModelSolver::IK_TYPE type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D; 

    std::vector<float> joints;
	if(armInUse->checkIk(pose, joints, type)){
		armInUse->joints(joints, true);
	}else{
		std::cout << "Not found IK" << std::endl;
	}

}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::ConnectBClicked(){
    QString qBackend;
    qBackend = ui->comboBox->currentText();
    mBackendArm = qBackend.toStdString();
    if(changeBackend(mBackendArm)){
        std::cout << "Changed backend" << std::endl;
    }
    else{
        std::cout << "NOT changed backend" << std::endl;
    }
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::ConnectAClicked(){
    if(ui->comboBox_2->currentText() == "Using Right Arm"){
        mUsingRight = true;
        armInUse = rightArm;
        std::cout << "USING RIGHT ARM" << std::endl;
    }
    else if(ui->comboBox_2->currentText() == "Using Left Arm"){
        mUsingRight = false;
        armInUse = leftArm;
        std::cout << "USING LEFT ARM" << std::endl;
    }
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::Stop_ClawClicked()
{
    std::cout << "Stop claw" <<std::endl;
    armInUse->stopClaw();
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::Close_ClawClicked()
{
    std::cout << "Close claw" <<std::endl;
    armInUse->closeClaw();
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::Open_ClawClicked()
{
    std::cout << "Open Claw" <<std::endl;
    armInUse->openClaw();
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::HomeClicked()
{
    std::cout << "Home" <<std::endl;
    armInUse->home();
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::checkBoxClicked(bool _checked)
{
    if(ui->checkBox->isChecked()){
        ui->lineEdit_p4->setVisible(1);
        ui->lineEdit_p5->setVisible(1);
        ui->lineEdit_p6->setVisible(1);
        ui->label_21->setVisible(1);
        ui->label_22->setVisible(1);
        ui->label_23->setVisible(1);
    }
    else{
        ui->lineEdit_p4->setVisible(0);
        ui->lineEdit_p5->setVisible(0);
        ui->lineEdit_p6->setVisible(0);
        ui->label_21->setVisible(0);
        ui->label_22->setVisible(0);
        ui->label_23->setVisible(0);
    }
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::Run_jointsClicked()
{
    std::cout << "Run Joints"<<std::endl;
    
    QString qj1, qj2, qj3, qj4, qj5;
    qj1 = ui->lineEdit_j1->text();
    qj2 = ui->lineEdit_j2->text();
    qj3 = ui->lineEdit_j3->text();
    qj4 = ui->lineEdit_j4->text();
    qj5 = ui->lineEdit_j5->text();

    float j1, j2, j3, j4, j5;
    j1 = qj1.toFloat()/180.0*M_PI;
    j2 = qj2.toFloat()/180.0*M_PI;
    j3 = qj3.toFloat()/180.0*M_PI;
    j4 = qj4.toFloat()/180.0*M_PI;
    j5 = qj5.toFloat()/180.0*M_PI;

    std::vector<float> joints;
    joints = {j1, j2, j3, j4, j5};
    armInUse->joints(joints, true);
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::Run_positionClicked()
{
    std::cout << "Run Position" <<std::endl;
    bool forceOri;
    forceOri = ui->checkBox->isChecked();
    QString qx, qy, qz, qdx, qdy, qdz;
    qx = ui->lineEdit_p1->text();
    qy = ui->lineEdit_p2->text();
    qz = ui->lineEdit_p3->text();

    float x, y, z, dx, dy, dz;
    x = qx.toFloat();
    y = qy.toFloat();
    z = qz.toFloat();

    Eigen::Matrix4f pose;
    pose = Eigen::Matrix4f::Identity();
    pose(0,3) = x;
    pose(1,3) = y;
    pose(2,3) = z;

    hecatonquiros::ModelSolver::IK_TYPE type;
    if(forceOri){
        qdx = ui->lineEdit_p4->text();
        qdy = ui->lineEdit_p5->text();
        qdz = ui->lineEdit_p6->text();

        dx = qdx.toDouble();
        dy = qdy.toDouble();
        dz = qdz.toDouble();

        Eigen::Vector3f zAxis = {dx, dy, dz};
		zAxis /=zAxis.norm();
		pose.block<3,1>(0,2) = zAxis;

        type = hecatonquiros::ModelSolver::IK_TYPE::IK_6D;
    }
    else{
        type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D;
    }   
    std::vector<float> joints;
	if(armInUse->checkIk(pose, joints, type)){
		armInUse->joints(joints, true);
	}else{
		std::cout << "Not found IK" << std::endl;
	}

}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::Run_autoposeClicked()
{
    auto pose = armInUse->pose();
    std::cout << "Arm Pose: " << std::endl;
    std::cout << pose << std::endl;

    ui->lineEdit_a1->setText(QString::number( pose(0,0) ));
    ui->lineEdit_a2->setText(QString::number( pose(0,1) ));
    ui->lineEdit_a3->setText(QString::number( pose(0,2) ));
    ui->lineEdit_a4->setText(QString::number( pose(0,3) ));
    ui->lineEdit_a5->setText(QString::number( pose(1,0) ));
    ui->lineEdit_a6->setText(QString::number( pose(1,1) ));
    ui->lineEdit_a7->setText(QString::number( pose(1,2) ));
    ui->lineEdit_a8->setText(QString::number( pose(1,3) ));
    ui->lineEdit_a9->setText(QString::number( pose(2,0) ));
    ui->lineEdit_a10->setText(QString::number( pose(2,1) ));
    ui->lineEdit_a11->setText(QString::number( pose(2,2) ));
    ui->lineEdit_a12->setText(QString::number( pose(2,3) ));
    ui->lineEdit_a13->setText(QString::number( pose(3,0) ));
    ui->lineEdit_a14->setText(QString::number( pose(3,1) ));
    ui->lineEdit_a15->setText(QString::number( pose(3,2) ));
    ui->lineEdit_a16->setText(QString::number( pose(3,3) ));

}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::Run_addWayPointClicked(){

    QString qx, qy, qz, qd;
    qx = ui->lineEdit_XAddWP->text();
    qy = ui->lineEdit_YAddWP->text();
    qz = ui->lineEdit_ZAddWP->text();
    qd = ui->lineEdit_DAddWP->text();

    float x, y, z, d;
    x = qx.toFloat();
    y = qy.toFloat();
    z = qz.toFloat();
    d = qd.toInt();

    std::vector<float> point = {x, y, z};
    mWayPoints.push_back(std::make_pair(point, d));

    std::string swaypoint = "X: " + std::to_string(x) + " , " +  "Y: " + std::to_string(y) + " , " + "Z: " + std::to_string(z) + " , " + "D: " + std::to_string(d);
    
    ui->listWidget_WayPoints->addItem(QString::fromStdString(swaypoint));

    
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::Run_WayPointsClicked(){

    for(int i = 0; i < mWayPoints.size(); i++ ){
        Eigen::Matrix4f pose;
        pose = Eigen::Matrix4f::Identity();
        pose(0,3) = mWayPoints[i].first[0];
        pose(1,3) = mWayPoints[i].first[1];
        pose(2,3) = mWayPoints[i].first[2];
    
        hecatonquiros::ModelSolver::IK_TYPE type;
        type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D;
    
        std::vector<float> joints;
	    if(armInUse->checkIk(pose, joints, type)){
	    	armInUse->joints(joints, true);
	    }else{
	    	std::cout << "Not found IK" << std::endl;
	    }
    
        std::this_thread::sleep_for(std::chrono::milliseconds(mWayPoints[i].second));
    }

    mWayPoints.clear();

    while(ui->listWidget_WayPoints->count() > 0)
    {
        delete ui->listWidget_WayPoints->takeItem(0);                        
    }

}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::Run_readPosLoad1Clicked(){
    QString qjoint;
    qjoint = ui->lineEdit_idread->text();
    
    int jointToRead;
    jointToRead = qjoint.toInt();
    
    int load = armInUse->readLoad(jointToRead);
    ui->lineEdit_load->setText(QString::number( load ));

    int pos = armInUse->readPos(jointToRead);
    ui->lineEdit_pos->setText(QString::number( pos ));
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::Run_readPosLoadCClicked(){
    mEndRead = true;
    int load = 0, pos = 0;
    mListenThread = std::thread([&]() {
        while(mEndRead){
            
            load = armInUse->readLoad(0);
            ui->lineEdit_load0->setText(QString::number( load ));
            pos = armInUse->readPos(0);
            ui->lineEdit_pos0->setText(QString::number( pos ));

            load = armInUse->readLoad(1);
            ui->lineEdit_load1->setText(QString::number( load ));
            pos = armInUse->readPos(1);
            ui->lineEdit_pos1->setText(QString::number( pos ));

            load = armInUse->readLoad(2);
            ui->lineEdit_load2->setText(QString::number( load ));
            pos = armInUse->readPos(2);
            ui->lineEdit_pos2->setText(QString::number( pos ));

            load = armInUse->readLoad(3);
            ui->lineEdit_load3->setText(QString::number( load ));
            pos = armInUse->readPos(3);
            ui->lineEdit_pos3->setText(QString::number( pos ));

            load = armInUse->readLoad(4);
            ui->lineEdit_load4->setText(QString::number( load ));
            pos = armInUse->readPos(4);
            ui->lineEdit_pos4->setText(QString::number( pos ));

            load = armInUse->readLoad(5);
            ui->lineEdit_load5->setText(QString::number( load ));
            pos = armInUse->readPos(5);
            ui->lineEdit_pos5->setText(QString::number( pos ));

            load = armInUse->readLoad(6);
            ui->lineEdit_load6->setText(QString::number( load ));
            pos = armInUse->readPos(6);
            ui->lineEdit_pos6->setText(QString::number( pos ));
            
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    });
    

}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::Stop_readPosLoadCClicked(){
    mEndRead = false; 
}



