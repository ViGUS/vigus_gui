#ifndef ARM_GUI_H
#define ARM_GUI_H

#include <QMainWindow>
#include <QStringList>
#include <QtGui>
#include <QtCore>

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

    void X1Clicked();

    void X2Clicked();

    void Y1Clicked();

    void Y2Clicked();

    void Z1Clicked();

    void Z2Clicked();

    void ConnectBClicked();

    void ConnectAClicked();

    void Stop_ClawClicked();

    void Close_ClawClicked();

    void Open_ClawClicked();

    void HomeClicked();
        
    void checkBoxClicked(bool _checked);

    void Run_jointsClicked();

    void Run_positionClicked();

    void Run_autoposeClicked();

    void Run_addWayPointClicked();

    void Run_WayPointsClicked();

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

    std::vector<std::pair<std::vector<float>, int>> mWayPoints;
};

#endif // ARM_GUI_H
