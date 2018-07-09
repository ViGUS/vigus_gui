#include "pclviewer_gui.h"
#include "ui_pclviewer_gui.h"

#include <CallbackSubscriber.h>

PCLViewer_gui::PCLViewer_gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCLViewer_gui)
    {

    ui->setupUi(this);
    this->setWindowTitle("PCL viewer");

    connect(ui->Delete_Spheres, SIGNAL(clicked()), this, SLOT(Delete_SpheresClicked()));

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
    std::string dir = "";
    for( int i = 0; i < _config.size(); i++){
        if( _config[i].first == "DirPCD"){
            mDirPCD = _config[i].second;
            dir = mDirPCD;
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
        if( _config[i].first == "DirTXT"){
            mDirTXT = _config[i].second;
            dir = mDirTXT;
        }
        if( _config[i].first == "DirPLY"){
            mDirPLY = _config[i].second;
            dir = mDirPLY;
        }
        if( _config[i].first == "Subscriber"){
            mNameSubscriber = _config[i].second;
            dir = mNameSubscriber;   
        }
    }

    if(dir == mDirTXT){
        extractPointCloud(mDirTXT);
    }else if(dir == mDirPCD){
        extractPointCloud(mDirPCD);
    }else if(dir == mDirPLY){
        extractPointCloud(mDirPLY);
    }else if(dir == mNameSubscriber){
        mSubPCL = new cbs::CallbackSubscriber<sensor_msgs::PointCloud2ConstPtr> (mNameSubscriber);
        std::cout << "Waiting to receive point cloud... " << std::endl;
        receivePointCloud(mNameSubscriber);
    }else{
        std::cout << "NO SUBSCRIBER, PTS, PCD or TXT Dir extracted!" << std::endl;    
    }
    
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::receivePointCloud(std::string _name){
    mEndSub = true;
    pcl::PointCloud<pcl::PointXYZRGB> cloudReceived;
    mListenThread = std::thread([&]() {
        while(mEndSub){
            if(mSubPCL->receiveData(cloudReceived)){
                mCloudT2.reset (new PointCloudT2(cloudReceived));
                mViewer->removePointCloud("cloudreceived");
                mViewer->addPointCloud(mCloudT2, "cloudreceived");
                ui->qvtkWidget->update(); 
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    });
}

//---------------------------------------------------------------------------------------------------------------------
bool PCLViewer_gui::extractPointCloud(std::string _dir)
{   
    // Set up the QVTK window
    mViewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow(mViewer->getRenderWindow());
    mViewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    if(_dir == mDirPCD){
        if(mTypePoint == "PointXYZ"){
            mCloudT1.reset (new PointCloudT1);
            if (pcl::io::loadPCDFile<PointT1>(_dir, *mCloudT1) == -1){
                PCL_ERROR ("Couldn't read file PCD \n");
                return false;
            }
            //std::cout << "Loaded PointCloud with Number of Points: " << mCloudT1->width*mCloudT1->height << std::endl;
            mViewer->addPointCloud(mCloudT1, "cloud");
        }else if(mTypePoint == "PointXYZRGB"){
            mCloudT2.reset (new PointCloudT2);
            if (pcl::io::loadPCDFile<PointT2>(_dir, *mCloudT2) == -1){
                PCL_ERROR ("Couldn't read file PCD \n");
                return false;
            }
            //std::cout << "Loaded PointCloud with Number of Points: " << mCloudT2->width*mCloudT2->height << std::endl;
            mViewer->addPointCloud(mCloudT2, "cloud");
        }
    }else if(_dir == mDirTXT){
        mCloudT1.reset (new PointCloudT1);
        mCloudT2.reset (new PointCloudT2);
        if(convertToPointCloud(mDirTXT)){
            if(mTypePoint == "PointXYZ"){
                //std::cout << "Loaded PointCloud with Number of Points: " << mCloudT1->width*mCloudT1->height << std::endl;
                mViewer->addPointCloud(mCloudT1, "cloud");
            }else if(mTypePoint == "PointXYZRGB"){
                //std::cout << "Loaded PointCloud with Number of Points: " << mCloudT2->width*mCloudT2->height << std::endl;
                mViewer->addPointCloud(mCloudT2, "cloud");
            }
        }
    }else if(_dir == mDirPLY){
        if(mTypePoint == "PointXYZ"){
            mCloudT1.reset (new PointCloudT1);
            if (pcl::io::loadPLYFile<PointT1>(_dir, *mCloudT1) == -1){
                PCL_ERROR ("Couldn't read file PLY \n");
                return false;
            }
            //std::cout << "Loaded PointCloud with Number of Points: " << mCloudT1->width*mCloudT1->height << std::endl;
            mViewer->addPointCloud(mCloudT1, "cloud");
        }else if(mTypePoint == "PointXYZRGB"){
            mCloudT2.reset (new PointCloudT2);
            if (pcl::io::loadPLYFile<PointT2>(_dir, *mCloudT2) == -1){
                PCL_ERROR ("Couldn't read file PLY \n");
                return false;
            }
            //std::cout << "Loaded PointCloud with Number of Points: " << mCloudT2->width*mCloudT2->height << std::endl;
            mViewer->addPointCloud(mCloudT2, "cloud");
        }
    }
    
    mViewer->resetCamera();
    mViewer->registerPointPickingCallback(&PCLViewer_gui::pointPickingOccurred, *this);
    ui->qvtkWidget->update();  

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool PCLViewer_gui::convertToPointCloud(std::string _dir)
{   
    std::string line_;
    std::ifstream file_(_dir);
    if(file_.is_open()){
        while(std::getline(file_, line_)){
            //std::getline(file_, line_);
            std::string x = "", y = "", z = "", dummy = "", r = "", g = "", b = "";
            int j = 0;
            int col = 1;
            while(line_.size() != j+1){
                switch(col){
                    case 1: //X
                        if(line_.at(j) != ' '){
                            x += line_.at(j);
                        }
                        break;
                    case 2: //Y
                        if(line_.at(j) != ' '){
                            y += line_.at(j);
                        }
                        break;
                    case 3: //Z
                        if(line_.at(j) != ' '){
                            z += line_.at(j);
                        }
                        break;
                    case 4:
                        if(line_.at(j) != ' '){
                            dummy += line_.at(j);
                        }
                        break;
                    case 5: //R
                        if(line_.at(j) != ' '){
                            r += line_.at(j);
                        }
                        break;
                    case 6: //G
                        if(line_.at(j) != ' '){
                            g += line_.at(j);
                        }
                        break;
                    case 7: //B
                        if(line_.at(j) != ' '){
                            b += line_.at(j);
                        }
                        break;
                    default:
                        break;
                }
                if(line_.at(j) != ' '){
                    j++;
                }else{
                    col++;
                    j++;
                }
            }
            if(x.size()>1 || y.size()>1 || z.size()>1){    
                PointT1 pclPoint;
                PointT2 pclPointRGB;

                float pX, pY, pZ, pR, pG, pB;
                std::stringstream ssX, ssY, ssZ, ssR, ssG, ssB;
                ssX << x; ssX >> pX;
                ssY << y; ssY >> pY;
                ssZ << z; ssZ >> pZ;
                ssR << r; ssR >> pR; 
                ssG << g; ssG >> pG;
                ssB << b; ssB >> pB;

                pclPoint.x = pX;
                pclPoint.y = pY;
                pclPoint.z = pZ;
                pclPointRGB.x = pX;
                pclPointRGB.y = pY;
                pclPointRGB.z = pZ;
                pclPointRGB.r = pR;
                pclPointRGB.g = pG;
                pclPointRGB.b = pB;
                
                mCloudT1->push_back(pclPoint);
                mCloudT2->push_back(pclPointRGB);
            }
        }
        file_.close();
    }else{
        std::cout << "File NOT open" << std::endl;
        return false;
    }

    //pcl::io::savePCDFileASCII("poindCloudRGB.pcd", *mCloudT2);
    //pcl::io::savePLYFileASCII("poindCloudRGB.ply", *mCloudT2);
    return true; 
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::updatePointCloudGUI()
{   
    if(mTypePoint == "PointXYZ"){
        mViewer->updatePointCloud(mCloudT1, "cloud");
    }else if(mTypePoint == "PointXYZRGB"){
        mViewer->updatePointCloud(mCloudT2, "cloud");
    }

    ui->qvtkWidget->update(); 
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::Delete_SpheresClicked(){
    
    for(int i = 0; i < mContSpheres; i++){
        std::string removeSphere = "sphere" + std::to_string(i);
        mViewer->removeShape(removeSphere);
    }
    mContSpheres = 0;
    updatePointCloudGUI();

}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::pointPickingOccurred(const pcl::visualization::PointPickingEvent &_event, void* _args){
    
    int idx = _event.getPointIndex();
    if(idx == -1){
        return;
    }

    float x, y, z;
    _event.getPoint(x, y, z);
    //std::cout << "Position (" << x << ", " << y << ", " << z << ")" << std::endl;

    QString qRadSphere;
    qRadSphere = ui->lineEdit_RadSphere->text();
    double radSphere;
    radSphere = qRadSphere.toDouble(); 

    std::string sSphere = "sphere" + std::to_string(mContSpheres);
    //std::cout << "sSphere: " << sSphere << std::endl;
    if(mTypePoint == "PointXYZ"){
        mViewer->addSphere(mCloudT1->points[idx], radSphere, 1, 0, 0.0, sSphere);
        updatePointCloudGUI();
    }else if(mTypePoint == "PointXYZRGB"){
        mViewer->addSphere(mCloudT2->points[idx], radSphere, 1, 0, 0.0, sSphere);
        updatePointCloudGUI();
    }
    mContSpheres++;

    ui->lineEdit_MX->setText(QString::number(x));
    ui->lineEdit_MY->setText(QString::number(y));
    ui->lineEdit_MZ->setText(QString::number(z));

}
