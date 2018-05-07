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
#include <QBrush>

#include "camera_gui.h"
#include "pclviewer_gui.h"
#include "arm_gui.h"
#include "uav_gui.h"

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

    void change1ColClicked();
    void change2ColClicked();
    void okeyClicked();
    void execWindows();
    void addRootGUI(QTreeWidgetItem *_item, QString _name, QString _description);
    void addChildGUI(QTreeWidgetItem *_parent, QString _name, QString _description);

private:
    Ui::Gui *ui;
    Arm_gui *arm_gui;
    UAV_gui *uav_gui;
    PCLViewer_gui *pclviewer_gui;
    Camera_gui *camera_gui;

    int mArgC;
    char **mArgV;

    std::vector<std::pair<std::string, std::string>> mExtractDataArm;
    std::vector<std::pair<std::string, std::string>> mExtractDataUAV;
    std::vector<std::pair<std::string, std::string>> mExtractDataPointCloud;
    std::vector<std::pair<std::string, std::string>> mExtractDataCamera;

    QTreeWidgetItem *mItemRootUAV;
    QTreeWidgetItem *mItemRootArm;
    QTreeWidgetItem *mItemRootPointCloud;
    QTreeWidgetItem *mItemRootCamera;
};

#endif // GUI_H