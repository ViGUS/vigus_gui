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
    ui->comboBox->addItem("No Backend");
    ui->comboBox->addItem("Feetech");
    ui->comboBox->addItem("Arduino");
    ui->comboBox_2->addItem("Using Right Arm");
    ui->comboBox_2->addItem("Using Left Arm");
    ui->checkBox->setChecked(false);

    connect(ui->Open_Claw, SIGNAL(clicked()), this, SLOT(on_Open_Claw_clicked()));
    connect(ui->Close_Claw, SIGNAL(clicked()), this, SLOT(on_Close_Claw_clicked()));
    connect(ui->Home, SIGNAL(clicked()), this, SLOT(on_Home_clicked()));
    connect(ui->Run_autopose, SIGNAL(clicked()), this, SLOT(on_Run_autopose_clicked()));
    connect(ui->Run_joints, SIGNAL(clicked()), this, SLOT(on_Run_joints_clicked()));
    connect(ui->Run_position, SIGNAL(clicked()), this, SLOT(on_Run_position_clicked()));
    connect(ui->Stop_Claw, SIGNAL(clicked()), this, SLOT(on_Stop_Claw_clicked()));
    connect(ui->checkBox, SIGNAL(clicked(bool)), this, SLOT(on_checkBox_clicked(bool)));

    connect(ui->comboBox_2, SIGNAL(currentIndexChanged(const QString)), this, SLOT(on_comboBox_2_currentIndexChanged(const QString)));
    connect(ui->comboBox, SIGNAL(currentIndexChanged(const QString)), this, SLOT(on_comboBox_currentIndexChanged(const QString)));

    }

//---------------------------------------------------------------------------------------------------------------------
Arm_gui::~Arm_gui()
{
    delete ui;
}

//---------------------------------------------------------------------------------------------------------------------
bool Arm_gui::configureGUI(std::vector<std::pair<std::string, std::string>> _config){

    for( int i = 0; i < _config.size(); i++){
        if( _config[i].first == "IdArm"){
            mIdArm = _config[i].second;
            ui->lineEdit_ID->setText(QString::fromStdString(mIdArm));
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

    }
     
    changeBackend(mBackendArm);
    return true;

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

        QString qSerialPort;
        qSerialPort = ui->lineEdit_serial->text();
        mSerialPortArm = qSerialPort.toStdString();
		serial::Serial* arduinoCom = new serial::Serial(mSerialPortArm, 115200, serial::Timeout::simpleTimeout(1000));
		if (!arduinoCom->isOpen()) {
			std::cout << "Could not open serial port" << std::endl;
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

        backendConfig1.configXML = "src/hecatonquiros/arm_controller/config/config_arm2.xml";
		backendConfig2.configXML = "src/hecatonquiros/arm_controller/config/config_arm1.xml";

		backendConfig1.type = hecatonquiros::Backend::Config::eType::Feetech; backendConfig1.port = mSerialPortArm; backendConfig1.armId =1;
		backendConfig2.type = hecatonquiros::Backend::Config::eType::Feetech; backendConfig2.port = mSerialPortArm; backendConfig2.armId =2;
		
    }else if(_backend == "No Backend"){
        backendConfig1.type = hecatonquiros::Backend::Config::eType::Dummy;
        backendConfig2.type = hecatonquiros::Backend::Config::eType::Dummy;
    }else{
        std::cout << "unrecognized mode, exiting" << std::endl;

      }


    modelSolverConfig1.type = hecatonquiros::ModelSolver::Config::eType::OpenRave;
    modelSolverConfig1.robotName = "left_arm";
    modelSolverConfig1.manipulatorName = "manipulator";
    if(mRobotFile != "" && mEnviromentFile == ""){
        modelSolverConfig1.robotFile = mRobotFile;
    }else if(mRobotFile == "" && mEnviromentFile != ""){
        modelSolverConfig1.environment = mEnviromentFile;
    }else if(mRobotFile == "" && mEnviromentFile == ""){
        std::cout << "ERROR! NO has puesto RobotFile o EnviromentFile" << std::endl;
    }else{
        std::cout << "ERROR! Has puesto RobotFile y EnviromentFile" << std::endl;
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

    armInUse = rightArm;
    mUsingRight = true;
    std::cout << "USING RIGHT ARM, start: " << std::endl;

}

//---------------------------------------------------------------------------------------------------------------------
void Arm_gui::on_comboBox_currentIndexChanged(const QString &_arg)
{
    QString qBackend;
    qBackend = ui->comboBox->currentText();
    mBackendArm = qBackend.toStdString();
    changeBackend(mBackendArm);

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
void Arm_gui::on_comboBox_2_currentIndexChanged(const QString &_arg)
{
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
void Arm_gui::on_checkBox_clicked(bool _checked)
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
void Arm_gui::on_Run_position_clicked()
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
void Arm_gui::on_Run_autopose_clicked()
{
    auto pose = armInUse->pose();
    std::cout << "Arm Pose: " << std::endl;
    std::cout << pose << std::endl;
    /*
    ui->lineEdit_a1->setText(QString::number( ));
    ui->lineEdit_a2->setText(QString::number( ));
    ui->lineEdit_a3->setText(QString::number( ));
    ui->lineEdit_a4->setText(QString::number( ));
    ui->lineEdit_a5->setText(QString::number( ));
    */
}
