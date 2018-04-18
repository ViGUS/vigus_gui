#include "pclviewer_gui.h"
#include "ui_pclviewer_gui.h"

PCLViewer_gui::PCLViewer_gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCLViewer_gui)
    {

    ui->setupUi(this);
    this->setWindowTitle("PCL viewer");

    }

//---------------------------------------------------------------------------------------------------------------------
PCLViewer_gui::~PCLViewer_gui()
{
    delete ui;
}

//---------------------------------------------------------------------------------------------------------------------
bool PCLViewer_gui::configureGUI(std::vector<std::pair<std::string, std::string>> _config)
{
    std::string type;
    for( int i = 0; i < _config.size(); i++){
      if( _config[i].first == "DirPCD"){
          mDirPCD = _config[i].second;
      }
      if( _config[i].first == "TypePoint"){
          type = _config[i].second;
          if(type == "PointXYZ"){
              mTypePoint = "PointXYZ";
          }
          else if(type == "PointXYZRGB"){
              mTypePoint = "PointXYZRGB";
          }
      }
    }

    extractPointCloud(mDirPCD);
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool PCLViewer_gui::extractPointCloud(std::string _dir)
{   
    // Set up the QVTK window
    mViewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow(mViewer->getRenderWindow());
    mViewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    if(mTypePoint == "PointXYZ"){
        mCloudT1.reset (new PointCloudT1);

        if (pcl::io::loadPCDFile<PointT1>(_dir, *mCloudT1) == -1){
          PCL_ERROR ("Couldn't read file PCD \n");
          return false;
        }

        std::cout << "Loaded PointCloud with Number of Points: " << mCloudT1->width*mCloudT1->height << std::endl;

        mViewer->addPointCloud(mCloudT1, "cloud");

    }else if(mTypePoint == "PointXYZRGB"){
        mCloudT2.reset (new PointCloudT2);

        if (pcl::io::loadPCDFile<PointT2>(_dir, *mCloudT2) == -1){
          PCL_ERROR ("Couldn't read file PCD \n");
          return false;
        }

        std::cout << "Loaded PointCloud with Number of Points: " << mCloudT2->width*mCloudT2->height << std::endl;

        mViewer->addPointCloud(mCloudT2, "cloud");

    }
    
    mViewer->resetCamera();
    ui->qvtkWidget->update();  

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::updatePointCloud()
{   
    if(mTypePoint == "PointXYZ"){
        mViewer->updatePointCloud(mCloudT1, "cloud");
    }else if(mTypePoint == "PointXYZRGB"){
        mViewer->updatePointCloud(mCloudT2, "cloud");
    }

    ui->qvtkWidget->update(); 
}
