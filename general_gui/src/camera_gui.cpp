#include "camera_gui.h"
#include "ui_camera_gui.h"
#include <QTextStream>

#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <chrono>

Camera_gui::Camera_gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Camera_gui)
    {

    ui->setupUi(this);

    

    }

//---------------------------------------------------------------------------------------------------------------------
Camera_gui::~Camera_gui()
{
    delete ui;
}

//---------------------------------------------------------------------------------------------------------------------
bool Camera_gui::configureGUI(std::vector<std::pair<std::string, std::string>> _config)
{
    for( int i = 0; i < _config.size(); i++){
        if( _config[i].first == "IPCamera"){
            mIPCamera = _config[i].second;
            ui->lineEdit_IP->setText(QString::fromStdString(mIPCamera));
        }
        if( _config[i].first == "PortCamera"){
            mPortCamera = _config[i].second;
            ui->lineEdit_Port->setText(QString::fromStdString(mPortCamera));
        }
    }
    
    return true;

}

//---------------------------------------------------------------------------------------------------------------------
bool Camera_gui::connectStreaming()
{

}

//---------------------------------------------------------------------------------------------------------------------
bool Camera_gui::runStreaming()
{

}

//---------------------------------------------------------------------------------------------------------------------
bool Camera_gui::putImage()
{

}

