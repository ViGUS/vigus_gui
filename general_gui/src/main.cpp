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

// rosrun general_gui general_gui src/vigus_gui/general_gui/config/gui_config.xml

int main(int _argc, char **_argv){
    ros::init(_argc, _argv, "general_gui");

    std::thread spinThread([&](){
    	ros::spin();
    });

    QApplication a(_argc, _argv);
    Gui gui;

    gui.extractData(_argv[1]);

    gui.show();
    
    return a.exec();
}
