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

    if(_argc < 2 || _argc > 2){
        std::cout << "ERROR! You must write: rosrun general_gui general_gui src/vigus_gui/general_gui/config/gui_config.xml" << std::endl;
        return 0;
    }

    ros::init(_argc, _argv, "general_gui");

    std::thread spinThread([&](){
    	ros::spin();
    });

    QApplication a(_argc, _argv);

    Gui gui;
    gui.extractData(_argv[1], _argc, _argv);
    gui.show();
    
    return a.exec();
}
