//
//
//
//
//
//

#include <QApplication>
#include "gui.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <ros/ros.h>

int main(int _argc, char **_argv){
    ros::init(_argc, _argv, "general_gui");

    std::thread spinThread([&](){
    	ros::spin();
    });

    QApplication a(_argc, _argv);
    Gui gui;

    gui.setReceived("PEPE");

    gui.show();
    
    return a.exec();
}
