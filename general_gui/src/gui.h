//
//
//
//
//
//

#ifndef GUI_H
#define GUI_H

#include <QMainWindow>
#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include "tinyxml2.h"
#include <QtCore>
#include <QtGui>
#include <QTreeWidgetItem>

#include "pclviewer_gui.h"
#include "arm_gui.h"
#include "uav_gui.h"
#include <argument_parser/argument_parser.h>

namespace Ui {
class Gui;
}

class Gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit Gui(QWidget *parent = 0);
    ~Gui();
    bool extractData(std::string _pathXML, int _argcCopy, char ** _argvCopy);

private slots:

    void okeyClicked();
    void execWindows();
    void addRoot(QTreeWidgetItem *_item, QString _name, QString _description);
    void addChild(QTreeWidgetItem *_parent, QString _name, QString _description);

private:
    Ui::Gui *ui;
    Arm_gui *arm_gui;
    UAV_gui *uav_gui;
    PCLViewer_gui *pclviewer_gui;
    
    static grvc::utils::ArgumentParser *mArgParser;

    std::vector<std::pair<std::string, std::string>> mExtractDataArm;
    std::vector<std::pair<std::string, std::string>> mExtractDataUAV;
    std::vector<std::pair<std::string, std::string>> mExtractDataPointCloud;

    QTreeWidgetItem *mItemRootUAV;
    QTreeWidgetItem *mItemRootArm;
    QTreeWidgetItem *mItemRootPointCloud;
};

#endif // GUI_H