#include "uav_gui.h"
#include "ui_uav_gui.h"
#include <QTextStream>

#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <fstream>

UAV_gui::UAV_gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::UAV_gui)
    {

    ui->setupUi(this);

    connect(ui->takeOff, SIGNAL(clicked()), this, SLOT(takeOffClicked()));
    connect(ui->land, SIGNAL(clicked()), this, SLOT(landClicked()));
    connect(ui->Run_pose, SIGNAL(clicked()), this, SLOT(Run_poseClicked()));
    connect(ui->Run_customPose, SIGNAL(clicked()), this, SLOT(Run_customPoseClicked()));
    connect(ui->Run_radiusCircle, SIGNAL(clicked()), this, SLOT(Run_radiusCircleClicked()));
    connect(ui->Run_radiusEight, SIGNAL(clicked()), this, SLOT(Run_radiusEightClicked()));
    connect(ui->Run_x, SIGNAL(clicked()), this, SLOT(Run_xClicked()));
    connect(ui->Run_y, SIGNAL(clicked()), this, SLOT(Run_yClicked()));
    connect(ui->Run_z, SIGNAL(clicked()), this, SLOT(Run_zClicked()));

    }

//---------------------------------------------------------------------------------------------------------------------
UAV_gui::~UAV_gui()
{
    delete ui;
}

//---------------------------------------------------------------------------------------------------------------------
bool UAV_gui::configureGUI(std::vector<std::pair<std::string, std::string>> _config, int _argcCopy, char ** _argvCopy)
{
    int argc;
    char **argv;
    for( int i = 0; i < _config.size(); i++){
        if( _config[i].first == "idUAV"){
            mIdUAV = _config[i].second;
            ui->lineEdit_ID->setText(QString::fromStdString(mIdUAV));
        }
    }
    
    // TODO: SET ID TO UAV FOR MULTIPLE UAV

    mUal = new grvc::ual::UAL(_argcCopy, _argvCopy);
    while (!mUal->isReady() && ros::ok()) {
        sleep(1);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    return true;

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::takeOffClicked()
{
    QString qTakeOff;
    qTakeOff = ui->lineEdit_takeoff->text();
    float takeOff;
    takeOff = qTakeOff.toFloat();
    std::cout << "TakeOff, with height: " << takeOff << std::endl;
    mUal->takeOff(takeOff);
    std::cout << "Finished TakeOff" << std::endl;
}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::landClicked()
{
    std::cout << "Landing..." <<std::endl;
    mUal->land(true);
    std::cout << "Landed!" <<std::endl;
}


//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::Run_xClicked()
{
    QString qStepX;
    qStepX = ui->lineEdit_sx->text();
    float stepX;
    stepX = qStepX.toFloat();
    std::cout << "Step in X: " << stepX << std::endl;

    mPose = mUal->pose();
    mPose.pose.position.x += stepX;
    mUal->goToWaypoint(mPose);
    std::cout << "Finished Step in X!" << std::endl;
}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::Run_yClicked()
{
    QString qStepY;
    qStepY = ui->lineEdit_sy->text();
    float stepY;
    stepY = qStepY.toFloat();
    std::cout << "Step in Y: " << stepY << std::endl;

    mPose = mUal->pose();
    mPose.pose.position.y += stepY;
    mUal->goToWaypoint(mPose);
    std::cout << "Finished Step in Y!" << std::endl;
}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::Run_zClicked()
{
    QString qStepZ;
    qStepZ = ui->lineEdit_sx->text();
    float stepZ;
    stepZ = qStepZ.toFloat();
    std::cout << "Step in Z: " << stepZ << std::endl;

    mPose = mUal->pose();
    mPose.pose.position.z += stepZ;
    mUal->goToWaypoint(mPose);
    std::cout << "Finished Step in Z!" << std::endl;
}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::Run_poseClicked()
{
    std::cout << "Show pose from UAL" << std::endl;
    mPose = mUal->pose();
    ui->lineEdit_j1->setText(QString::number(mPose.pose.position.x));
    ui->lineEdit_j2->setText(QString::number(mPose.pose.position.y));
    ui->lineEdit_j3->setText(QString::number(mPose.pose.position.z));

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::Run_customPoseClicked()
{   
    QString qX, qY, qZ;
    qX = ui->lineEdit_a1->text();
    qY = ui->lineEdit_a2->text();
    qZ = ui->lineEdit_a3->text();

    float x, y, z;
    x = qX.toFloat();
    y = qY.toFloat();
    z = qZ.toFloat();

    mPose = mUal->pose();
    std::cout << "You wrote: " << x << ", " << y << ", " << z << std::endl;
    std::cout << "And current pose is: " << mPose.pose.position.x << ", " << mPose.pose.position.y << ", " << mPose.pose.position.z << std::endl;

    std::cout << "Moving..."<< std::endl;
    auto targetPose = mUal->pose();
    targetPose.pose.position.x = x;
    targetPose.pose.position.y = y;
    targetPose.pose.position.z = z;
    mUal->goToWaypoint(targetPose);

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::Run_radiusEightClicked()
{
    QString qRadius;
    qRadius = ui->lineEdit_re1->text();
    float radius;
    radius = qRadius.toFloat();

    std::vector<geometry_msgs::PoseStamped> poses;
    mPose = mUal->pose();
    for(unsigned i = 0; i < 16; i++){
        auto eigPose = mPose;
        eigPose.pose.position.x += radius*sin(2*M_PI/16*i);
        eigPose.pose.position.y += radius*sin(2*M_PI/16*i)*cos(2*M_PI/16*i);
        poses.push_back(eigPose);
    }
    for(auto &pos: poses){
        mUal->goToWaypoint(pos);
    }
}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::Run_radiusCircleClicked()
{   
    QString qRadius;
    qRadius = ui->lineEdit_rc1->text();
    float radius;
    radius = qRadius.toFloat();

    std::vector<geometry_msgs::PoseStamped> poses;
    mPose = mUal->pose();
    for(unsigned i = 0; i < 16; i++){
        auto cirPose = mPose;
        cirPose.pose.position.x += radius*cos(2*M_PI/16*i);
        cirPose.pose.position.y += radius*sin(2*M_PI/16*i);
        poses.push_back(cirPose);
    }
    std::cout << "Running Radius of circle" << std::endl;
    for(auto &pos: poses){
        mUal->goToWaypoint(pos);
    }
    std::cout << "Finished Radius of circle!" << std::endl;
}
