//
//
//
//
//
//

#include "gui.h"
#include "ui_gui.h"

#include <QTextStream>
#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <chrono>


Gui::Gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Gui)
    {

    ui->setupUi(this);
    connect(ui->exit_button, SIGNAL(clicked()), this, SLOT(on_Stop_clicked()));
    }

Gui::~Gui()
{
    delete ui;
}

void Gui::setReceived(const QString &received)
{
  ui->lineEdit_received->setText(received);
}
 
QString Gui::received() const
{
    return ui->lineEdit_received->text();
}

void Gui::on_Stop_clicked()
{
    if( ui->lineEdit_to_send->text() == "1"){
        std::cout << "Stop program" <<std::endl;
    }
    else{
        QString data = ui->lineEdit_to_send->text();
        std::cout << "Has puesto: " << data.toStdString() << std::endl;
    }

}
