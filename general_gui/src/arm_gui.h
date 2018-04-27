#ifndef ARM_GUI_H
#define ARM_GUI_H

#include <QMainWindow>

#include <hecatonquiros/Positioner.h>
#include <hecatonquiros/Arm4DoF.h>

namespace Ui {
class Arm_gui;
}

class Arm_gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit Arm_gui(QWidget *parent = 0);
    ~Arm_gui();

    bool configureGUI(std::vector<std::pair<std::string, std::string>> _config);

private slots:

    bool changeBackend(std::string _backend);

    void on_X1_clicked();

    void on_X2_clicked();

    void on_Y1_clicked();

    void on_Y2_clicked();

    void on_Z1_clicked();

    void on_Z2_clicked();

    void on_ConnectB_clicked();

    void on_ConnectA_clicked();

    void on_Stop_Claw_clicked();

    void on_Close_Claw_clicked();

    void on_Open_Claw_clicked();

    void on_Home_clicked();

    void on_checkBox_clicked(bool _checked);

    void on_Run_joints_clicked();

    void on_Run_position_clicked();

    void on_Run_autopose_clicked();

private:
    Ui::Arm_gui *ui;

    hecatonquiros::Backend::Config backendConfig1;
    hecatonquiros::Backend::Config backendConfig2;
    hecatonquiros::ModelSolver::Config modelSolverConfig1;
    hecatonquiros::ModelSolver::Config modelSolverConfig2;
    hecatonquiros::Arm4DoF *leftArm;
    hecatonquiros::Arm4DoF *rightArm;
    hecatonquiros::Arm4DoF *armInUse;
    bool mUsingRight;

    std::string mIdArm;
    std::string mBackendArm;
    std::string mSerialPortArm;
    std::string mEnviromentFile = "";
    std::string mRobotFile = "";
    bool mVisualizer;
};

#endif // ARM_GUI_H
