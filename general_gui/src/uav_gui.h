#ifndef UAV_GUI_H
#define UAV_GUI_H

#include <QMainWindow>

#include <uav_abstraction_layer/ual.h>
#include <argument_parser/argument_parser.h>

namespace Ui {
class UAV_gui;
}

class UAV_gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit UAV_gui(QWidget *parent = 0);
    ~UAV_gui();

    bool configureGUI(std::vector<std::pair<std::string, std::string>> _config, grvc::utils::ArgumentParser *_argParser);

private slots:

    void on_land_clicked();

    void on_takeOff_clicked();

    void on_Run_x_clicked();

    void on_Run_y_clicked();

    void on_Run_z_clicked();

    void on_Run_pose_clicked();

    void on_Run_customPose_clicked();

    void on_Run_radiusEight_clicked();

    void on_Run_radiusCircle_clicked();

private:
    Ui::UAV_gui *ui;

    grvc::ual::UAL *mUal;
    geometry_msgs::PoseStamped mPose;
    std::string mIdUAV;
    
};

#endif // UAV_GUI_H
