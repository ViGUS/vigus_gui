#ifndef PCLVIEWER_GUI_H
#define PCLVIEWER_GUI_H

#include <iostream>
#include <QMainWindow>
#include <fstream>
#include <string>
#include <sstream>
#include <thread>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/PoseStamped.h"

typedef pcl::PointXYZ PointT1;
typedef pcl::PointCloud<PointT1> PointCloudT1;

typedef pcl::PointXYZRGB PointT2;
typedef pcl::PointCloud<PointT2> PointCloudT2;

namespace cbs{
    template<typename T_>
    class CallbackSubscriber;
}

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

    void receivePointCloud(std::string _name);

    void updatePointCloudGUI();

    bool extractPointCloud(std::string _dir);

    bool convertToPointCloud(std::string _dir);

    void Delete_SpheresClicked();

    void pointPickingOccurred(const pcl::visualization::PointPickingEvent &_event, void* _args);

    void CallbackPose(const geometry_msgs::PoseStamped::ConstPtr& _msg);

    void addPose(std::string _name);

    void updatePose(std::string _name);

private:
    Ui::PCLViewer_gui *ui;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
    PointCloudT1::Ptr mCloudT1;
    PointCloudT2::Ptr mCloudT2;

    std::string mDirPCD = "";
    std::string mDirTXT = "";
    std::string mDirPLY = "";
    std::string mTypePoint;
    std::string mNameSubscriber = "";
    std::string mNameCallbackPose = "";

    int mContSpheres = 1;
    bool mEndSub = false;

    cbs::CallbackSubscriber<sensor_msgs::PointCloud2ConstPtr> *mSubPCL;
    ros::Subscriber mPoseSubscriber;

    float mPoseX = 0.0, mPoseY = 0.0, mPoseZ = 0.0, mPoseOX = 0.0, mPoseOY = 0.0, mPoseOZ = 0.0;

    std::thread mListenThread, mUpdatePoseThread;
};


#endif // PCLVIEWER_GUI_H
