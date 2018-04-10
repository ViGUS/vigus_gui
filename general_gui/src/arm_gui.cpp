#include "arm_gui.h"
#include "ui_arm_gui.h"
#include <QTextStream>

#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <chrono>

Arm_gui::Arm_gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Arm_gui)
{

    ui->setupUi(this);
    ui->comboBox_2->addItem("Using Right Arm");
    ui->comboBox_2->addItem("Using Left Arm");
    ui->checkBox->setChecked(true);
    ui->lineEdit_serial->setVisible(1);

    connect(ui->Open_Claw, SIGNAL(clicked()), this, SLOT(on_Open_Claw_clicked()));
    connect(ui->Close_Claw, SIGNAL(clicked()), this, SLOT(on_Close_Claw_clicked()));
    connect(ui->Home, SIGNAL(clicked()), this, SLOT(on_Home_clicked()));
    connect(ui->Run_autopose, SIGNAL(clicked()), this, SLOT(on_Run_autopose_clicked()));
    connect(ui->Run_joints, SIGNAL(clicked()), this, SLOT(on_Run_joints_clicked()));
    connect(ui->Run_position, SIGNAL(clicked()), this, SLOT(on_Run_position_clicked()));
    connect(ui->Stop_Claw, SIGNAL(clicked()), this, SLOT(on_Stop_Claw_clicked()));
    connect(ui->checkBox, SIGNAL(clicked()), this, SLOT(on_checkBox_clicked(bool)));
    connect(ui->Change_Backend, SIGNAL(clicked()), this, SLOT(on_Change_Backend_clicked()));

    // TODO: CHECK THIS
    connect(ui->comboBox_2, SIGNAL(currentIndexChanged(const QString)), this, SLOT(on_comboBox_2_currentIndexChanged(const QString));

}

//---------------------------------------------------------------------------------------------------------------------
Arm_gui::~Arm_gui()
{
    delete ui;
}

//---------------------------------------------------------------------------------------------------------------------
bool Arm_gui::configureGUI(std::vector<std::pair<std::string,double>> _config){

    // TODO: EXTRACT DATA FROM GENERAL GUI




}
//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::changeBackend(std::string _backend){

    if(_backend == "Gazebo"){

        backendConfig1.type = hecatonquiros::Backend::Config::eType::Gazebo;
        backendConfig1.topic = "left_arm/joint_states";
        backendConfig1.armId =1;
        backendConfig2.type = hecatonquiros::Backend::Config::eType::Gazebo;
        backendConfig2.topic = "right_arm/joint_states";
        backendConfig2.armId =2;

    }else if(_backend == "Arduino"){

        std::string serialPort;
        QString qSerialPort;
        qSerialPort = ui->lineEdit_serial->text();
        serialPort = qSerialPort.toStdString();
		serial::Serial* arduinoCom = new serial::Serial(serialPort, 115200, serial::Timeout::simpleTimeout(1000));
		if (!arduinoCom->isOpen()) {
			std::cout << "Could not open serial port" << std::endl;
		}

		backendConfig1.type = hecatonquiros::Backend::Config::eType::Arduino; backendConfig1.sharedSerialPort = arduinoCom; backendConfig1.armId =1;
		backendConfig2.type = hecatonquiros::Backend::Config::eType::Arduino; backendConfig2.sharedSerialPort = arduinoCom; backendConfig2.armId =2;
    
    }else if(_backend == "Feetech"){

        std::string serialPort;
        QString qSerialPort;
        qSerialPort = ui->lineEdit_serial->text();
        serialPort = qSerialPort.toStdString();

		backendConfig1.valuesMinMax = { {-109.376, 106.492},  
                                    	{-111.490, 107.986},
                                    	{-104.869, 113.775},
                                    	{-111.568, 98.624},
                                    	{-152.400, 144.563},
                                    	{-135.305, 157.744}};

		backendConfig2.valuesMinMax = { {-112.737, 100.442},  
                                        {-106.202, 110.798},
                                        {-103.401, 116.820},
                                        {-92.791, 120.046},
                                        {-152.098, 136.935},
                                        {-147.468, 147.564}};

		backendConfig1.type = hecatonquiros::Backend::Config::eType::Feetech; backendConfig1.port = serialPort; backendConfig1.armId =1;
		backendConfig1.jointsOffsets = { 0,	0,	0, 0,	0,	0 };
		backendConfig2.type = hecatonquiros::Backend::Config::eType::Feetech; backendConfig2.port = serialPort; backendConfig2.armId =2;
		backendConfig2.jointsOffsets = { 0,	0,	0,-15.0/180.0*M_PI,	0,	-15.0/180.0*M_PI};
   
    }else if(_backend == "No Backend"){
        backendConfig1.type = hecatonquiros::Backend::Config::eType::Dummy;
        backendConfig2.type = hecatonquiros::Backend::Config::eType::Dummy;
    }else{
        std::cout << "unrecognized mode, exiting" << std::endl;

      }


    modelSolverConfig1.type = hecatonquiros::ModelSolver::Config::eType::Simple4DoF;
    modelSolverConfig1.robotName = "left_arm";
    modelSolverConfig1.manipulatorName = "manipulator";
    //modelSolverConfig1.robotFile = _argv[1];
    modelSolverConfig1.environment = _argv[1];
    modelSolverConfig1.offset = {0.20,0.14,-0.04};
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(0*M_PI, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(1*M_PI,  Eigen::Vector3f::UnitY());
	Eigen::Quaternionf q(m);
    modelSolverConfig1.rotation = {q.w(), q.x(), q.y(), q.z()};
    modelSolverConfig1.visualizer = true;

    hecatonquiros::ModelSolver::Config modelSolverConfig2;
    modelSolverConfig2.type = hecatonquiros::ModelSolver::Config::eType::Simple4DoF;
    modelSolverConfig2.robotName = "right_arm";
    modelSolverConfig2.manipulatorName = "manipulator";
    //modelSolverConfig2.robotFile = _argv[1];
    modelSolverConfig2.offset = {0.2,-0.2,-0.04};
    modelSolverConfig2.rotation = {q.w(), q.x(), q.y(), q.z()};
    modelSolverConfig2.visualizer = true;

    leftArm = new hecatonquiros::Arm4DoF (modelSolverConfig1, backendConfig1);
    rightArm = new hecatonquiros::Arm4DoF (modelSolverConfig2, backendConfig2);

    leftArm->home();
    rightArm->home();

    armInUse = rightArm;
    usingRight = true;
    std::cout << "USING RIGHT ARM, start: " << std::endl;

}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::on_Change_Backend_clicked(){

    std::string stringBackend;
    QString qBackend;
    qBackend = ui->lineEdit_Backend->text();
    stringBackend = qBackend.toStdString();
    changeBackend(stringBackend);

}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::on_Stop_Claw_clicked()
{
    std::cout << "Stop claw" <<std::endl;
    armInUse->stopClaw();
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::on_Close_Claw_clicked()
{
    std::cout << "Close claw" <<std::endl;
    armInUse->closeClaw();
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::on_Open_Claw_clicked()
{
    std::cout << "Open Claw" <<std::endl;
    armInUse->openClaw();
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::on_Home_clicked()
{
    std::cout << "Home" <<std::endl;
    armInUse->home();
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::on_comboBox_2_currentIndexChanged(const QString &arg1)
{
    if(ui->comboBox_2->currentText() == "Using Right Arm"){
        usingRight = true;
        armInUse = rightArm;
        std::cout << "USING RIGHT ARM" << std::endl;
    }
    else if(ui->comboBox_2->currentText() == "Using Left Arm"){
        usingRight = false;
        armInUse = leftArm;
        std::cout << "USING LEFT ARM" << std::endl;
    }
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::on_checkBox_clicked(bool checked)
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
void Arm_gui::on_Run_joints_clicked()
{
    float j1, j2, j3, j4, j5;
    std::cout << "Joints: "<<std::endl;
    std::vector<float> joints;
    QString qj1, qj2, qj3, qj4, qj5;

    qj1 = ui->lineEdit_j1->text();
    qj2 = ui->lineEdit_j2->text();
    qj3 = ui->lineEdit_j3->text();
    qj4 = ui->lineEdit_j4->text();
    qj5 = ui->lineEdit_j5->text();

    j1 = qj1.toFloat();
    j2 = qj2.toFloat();
    j3 = qj3.toFloat();
    j4 = qj4.toFloat();
    j5 = qj5.toFloat();

    armInUse->joints({j1/180.0*M_PI,j2/180.0*M_PI,j3/180.0*M_PI,j4/180.0*M_PI,j5/180.0*M_PI});
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::on_Run_position_clicked()
{
    Eigen::Matrix4f pose;
    std::vector<float> joints;
    std::cout << "position" <<std::endl;
    float x, y, z, dx, dy, dz;
    bool forceOri;
    forceOri = ui->checkBox->isChecked();
    QString qx, qy, qz, qdx, qdy, qdz;
    qx = ui->lineEdit_p1->text();
    qy = ui->lineEdit_p2->text();
    qz = ui->lineEdit_p3->text();

    x = qx.toFloat();
    y = qy.toFloat();
    z = qz.toFloat();

    pose = Eigen::Matrix4f::Identity();
    pose(0,3) = x;
    pose(1,3) = y;
    pose(2,3) = z;
    if(ui->checkBox->isChecked()){
        qdx = ui->lineEdit_p4->text();
        qdy = ui->lineEdit_p5->text();
        qdz = ui->lineEdit_p6->text();

        dx = qdx.toDouble();
        dy = qdy.toDouble();
        dz = qdz.toDouble();
        pose(0,2) = dx;
        pose(1,2) = dy;
        pose(2,2) = dz;
    }
    if(armInUse->checkIk(pose, joints, forceOri)){
        armInUse->joints(joints);
    }else{
        std::cout << "Not found IK" << std::endl;
    }
}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::on_Run_autopose_clicked()
{
    auto pose = armInUse->pose();
    std::cout << pose << std::endl;
    // TODO: PUT POSE IN TEXT

}
