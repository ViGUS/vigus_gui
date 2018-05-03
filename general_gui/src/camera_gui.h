#ifndef CAMERA_GUI_H
#define CAMERA_GUI_H

#include <QMainWindow>


namespace Ui {
    class Camera_gui;
}

class Camera_gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit Camera_gui(QWidget *parent = 0);
    ~Camera_gui();

    bool configureGUI(std::vector<std::pair<std::string, std::string>> _config);

private slots:
    bool connectStreaming();
    bool runStreaming();
    void putImage();

private:
    Ui::Camera_gui *ui;

    std::string mIPCamera;
    std::string mPortCamera;
    
};

#endif // CAMERA_GUI_H
