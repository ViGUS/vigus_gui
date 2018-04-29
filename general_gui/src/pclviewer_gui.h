#ifndef PCLVIEWER_GUI_H
#define PCLVIEWER_GUI_H

#include <iostream>
#include <QMainWindow>
#include <fstream>
#include <string>
#include <sstream>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>

typedef pcl::PointXYZ PointT1;
typedef pcl::PointCloud<PointT1> PointCloudT1;

typedef pcl::PointXYZRGB PointT2;
typedef pcl::PointCloud<PointT2> PointCloudT2;

namespace Ui
{
    class PCLViewer_gui;
}

class PCLViewer_gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit PCLViewer_gui(QWidget *parent = 0);
    ~PCLViewer_gui ();

    bool configureGUI(std::vector<std::pair<std::string, std::string>> _config);

private slots:

    void updatePointCloudGUI();

    bool extractPointCloud(std::string _dir);

    bool convertToPointCloud(std::string _dir);

    void on_Delete_Spheres_clicked();

    void pointPickingOccurred(const pcl::visualization::PointPickingEvent &_event, void* _args);

private:
    Ui::PCLViewer_gui *ui;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
    PointCloudT1::Ptr mCloudT1;
    PointCloudT2::Ptr mCloudT2;
    std::string mDirPCD = "";
    std::string mDirTXT = "";
    std::string mDirPLY = "";
    std::string mTypePoint;
    int mContSpheres = 0;
};

#endif // PCLVIEWER_GUI_H
