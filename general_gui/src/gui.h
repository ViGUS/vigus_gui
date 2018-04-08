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

namespace Ui {
class Gui;
}

class Gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit Gui(QWidget *parent = 0);
    ~Gui();
    bool extractData(std::string _pathXML);

private slots:

    void okeyClicked();
    void execWindows();
    void addRoot(QTreeWidgetItem *_item, QString _name, QString _description);
    void addChild(QTreeWidgetItem *_parent, QString _name, QString _description);

private:
    Ui::Gui *ui;
    QTreeWidgetItem *itemRootUAV;
    QTreeWidgetItem *itemRootArm;
};

#endif // GUI_H