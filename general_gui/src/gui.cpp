//
//
//
//
//
//

#include "gui.h"
#include "ui_gui.h"

#include <QTextStream>

Gui::Gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Gui)
    {

    ui->setupUi(this);
    connect(ui->okey_button, SIGNAL(clicked()), this, SLOT(okeyClicked()));
    connect(ui->change_button_1Col, SIGNAL(clicked()), this, SLOT(change1ColClicked()));
    connect(ui->change_button_2Col, SIGNAL(clicked()), this, SLOT(change2ColClicked()));

    ui->treeWidget->setColumnCount(2);
    ui->treeWidget->setHeaderLabels(QStringList() << "Windows" << "Data");

    }

//---------------------------------------------------------------------------------------------------------------------
Gui::~Gui()
{
    delete ui;
}

//---------------------------------------------------------------------------------------------------------------------
bool Gui::extractData(std::string _pathXML, int _argcCopy, char ** _argvCopy)
{   
    mArgParser = new grvc::utils::ArgumentParser(_argcCopy, _argvCopy);

    tinyxml2::XMLDocument xml_doc;
    //std::cout << "XML Path: " << _pathXML << std::endl; 
    tinyxml2::XMLError resultXML = xml_doc.LoadFile(_pathXML.c_str());
    if(resultXML != tinyxml2::XML_SUCCESS){ 
        std::cout << "Error loading file" << std::endl; 
        std::cout << resultXML << std::endl; 
        return false;
    }

    tinyxml2::XMLNode* rootWindows = xml_doc.FirstChildElement("Windows");
    if(rootWindows == nullptr){
        std::cout << "Error NO Windows root" << std::endl; 
        return false;
    }
    else{   
        // READ DATA FROM UAV IF EXIST 
        int contUAV = 0;
        for (tinyxml2::XMLElement* childUAV = rootWindows->FirstChildElement("UAV"); childUAV != NULL; childUAV = childUAV->NextSiblingElement("UAV")){
            contUAV++;  

            // Add Root to GUI
            mItemRootUAV = new QTreeWidgetItem(ui->treeWidget);
            addRootGUI(mItemRootUAV, "UAV", QString::number(contUAV));

            tinyxml2::XMLElement* itemUAV = childUAV->FirstChildElement("idUAV");
            int idUAV;
            resultXML = itemUAV->QueryIntText(&idUAV);
            if(resultXML != tinyxml2::XML_SUCCESS){
                std::cout << "Error extracting id UAV" << std::endl;   
            }
            else{
                // Add Child for UAV Root
                addChildGUI(mItemRootUAV, "idUAV", QString::number(idUAV));
                mExtractDataUAV.push_back(std::make_pair("idUAV", std::to_string(idUAV)));
            }
               
        }

        // READ DATA FROM ARM IF EXIST 
        int contArm = 0;
        for (tinyxml2::XMLElement* childArm = rootWindows->FirstChildElement("Arm"); childArm != NULL; childArm = childArm->NextSiblingElement("Arm")){
            contArm++;

            // Add Root to GUI
            mItemRootArm = new QTreeWidgetItem(ui->treeWidget);
            addRootGUI(mItemRootArm, "Arm", QString::number(contArm));

            tinyxml2::XMLElement* itemArm = childArm->FirstChildElement("idArm");
            int idArm;
            resultXML = itemArm->QueryIntText(&idArm);
            addChildGUI(mItemRootArm, "idArm", QString::number(idArm));
            mExtractDataArm.push_back(std::make_pair("idArm", std::to_string(idArm)));

            itemArm = childArm->FirstChildElement("SerialPort");
            std::string serialPort;
            serialPort = itemArm->GetText();
            // Add Child for Arm Root
            addChildGUI(mItemRootArm, "SerialPort", QString::fromStdString(serialPort));
            mExtractDataArm.push_back(std::make_pair("SerialPort", serialPort));

            itemArm = childArm->FirstChildElement("RobotFile");
            if(itemArm){
                std::string robotFile = "";
                robotFile = itemArm->GetText();
                // Add Child for Arm Root
                addChildGUI(mItemRootArm, "RobotFile", QString::fromStdString(robotFile));
                mExtractDataArm.push_back(std::make_pair("RobotFile", robotFile));
            }

            itemArm = childArm->FirstChildElement("EnviromentFile");
            if(itemArm){
                std::string enviromentFile = "";
                enviromentFile = itemArm->GetText();
                // Add Child for Arm Root
                addChildGUI(mItemRootArm, "EnviromentFile", QString::fromStdString(enviromentFile));
                mExtractDataArm.push_back(std::make_pair("EnviromentFile", enviromentFile));
            }
            
            itemArm = childArm->FirstChildElement("Visualizer");
            std::string visualizer;
            visualizer = itemArm->GetText();
            // Add Child for Arm Root
            addChildGUI(mItemRootArm, "Visualizer", QString::fromStdString(visualizer));
            mExtractDataArm.push_back(std::make_pair("Visualizer", visualizer));

            itemArm = childArm->FirstChildElement("Backend");
            std::string backend;
            backend = itemArm->GetText();
            // Add Child for Arm Root
            addChildGUI(mItemRootArm, "Backend", QString::fromStdString(backend));
            mExtractDataArm.push_back(std::make_pair("Backend", backend));
        }

        // READ DATA FROM POINTCLOUD IF EXIST 
        int contPointCloud = 0;
        for (tinyxml2::XMLElement* childPointCloud = rootWindows->FirstChildElement("PointCloud"); childPointCloud != NULL; childPointCloud = childPointCloud->NextSiblingElement("PointCloud")){
            contPointCloud++;

            // Add Root to GUI
            mItemRootPointCloud = new QTreeWidgetItem(ui->treeWidget);
            addRootGUI(mItemRootPointCloud, "PointCloud", QString::number(contPointCloud));

            tinyxml2::XMLElement* itemPointCloud = childPointCloud->FirstChildElement("DirPCD");
            if(itemPointCloud){
                std::string dirPCD;
                dirPCD = itemPointCloud->GetText();
                // Add Child for PointCloud Root
                addChildGUI(mItemRootPointCloud, "DirPCD", QString::fromStdString(dirPCD));
                mExtractDataPointCloud.push_back(std::make_pair("DirPCD", dirPCD));
            }
            
            itemPointCloud = childPointCloud->FirstChildElement("DirTXT");
            if(itemPointCloud){
                std::string dirTXT;
                dirTXT = itemPointCloud->GetText();
                // Add Child for PointCloud Root
                addChildGUI(mItemRootPointCloud, "DirTXT", QString::fromStdString(dirTXT));
                mExtractDataPointCloud.push_back(std::make_pair("DirTXT", dirTXT));
            }

            itemPointCloud = childPointCloud->FirstChildElement("DirPLY");
            if(itemPointCloud){
                std::string dirPLY;
                dirPLY = itemPointCloud->GetText();
                // Add Child for PointCloud Root
                addChildGUI(mItemRootPointCloud, "DirPLY", QString::fromStdString(dirPLY));
                mExtractDataPointCloud.push_back(std::make_pair("DirPLY", dirPLY));
            }

            itemPointCloud = childPointCloud->FirstChildElement("TypePoint");
            std::string typePoint;
            typePoint = itemPointCloud->GetText();
            // Add Child for PointCloud Root
            addChildGUI(mItemRootPointCloud, "TypePoint", QString::fromStdString(typePoint));
            mExtractDataPointCloud.push_back(std::make_pair("TypePoint", typePoint));

            itemPointCloud = childPointCloud->FirstChildElement("Subscriber");
            if(itemPointCloud){
                std::string subscriber;
                subscriber = itemPointCloud->GetText();
                // Add Child for PointCloud Root
                addChildGUI(mItemRootPointCloud, "Subscriber", QString::fromStdString(subscriber));
                mExtractDataPointCloud.push_back(std::make_pair("Subscriber", subscriber));
            }
        }

        int contCamera++;
        for (tinyxml2::XMLElement* childCamera = rootWindows->FirstChildElement("Camera"); childCamera != NULL; childCamera = childCamera->NextSiblingElement("Camera")){
            contCamera++;  

            // Add Root to GUI
            mItemRootCamera = new QTreeWidgetItem(ui->treeWidget);
            addRootGUI(mItemRootCamera, "Camera", QString::number(contCamera));

            tinyxml2::XMLElement* itemCamera = childCamera->FirstChildElement("IPCamera");
            std::string ipCamera;
            ipCamera = itemCamera->GetText();
            addChildGUI(mItemRootCamera, "IPCamera", QString::fromStdString(ipCamera));
            mExtractDataCamera.push_back(std::make_pair("IPCamera", ipCamera));

            itemCamera = childCamera->FirstChildElement("idArm");
            int portCamera;
            resultXML = childCamera->QueryIntText(&portCamera);
            addChildGUI(mItemRootCamera, "PortCamera", QString::number(portCamera));
            mExtractDataCamera.push_back(std::make_pair("PortCamera", std::to_string(portCamera)));
               
        }

    }

    return true;

}

//---------------------------------------------------------------------------------------------------------------------
void Gui::okeyClicked()
{
    std::cout << "Thats okey for you, continue!" << std::endl;
    execWindows();

    //for(int i = 0; i < mExtractDataArm.size(); i++){
    //    std::cout << "Arm: " << mExtractDataArm[i].first << " | " << mExtractDataArm[i].second << std::endl;
    //    
    //} 
    //for(int i = 0; i < mExtractDataUAV.size(); i++){
    //    std::cout << "UAV: " << mExtractDataUAV[i].first << " | " << mExtractDataUAV[i].second << std::endl;
    //}
    //for(int i = 0; i < mExtractDataPointCloud.size(); i++){
    //    std::cout << "PointCloud: " << mExtractDataPointCloud[i].first << " | " << mExtractDataPointCloud[i].second << std::endl;
    //}
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::change1ColClicked()
{
    QBrush brush_green(Qt::green);
    ui->treeWidget->currentItem()->setBackground(0, brush_green);

    QString qOld = ui->treeWidget->currentItem()->text(0);
    std::string sOld = qOld.toStdString();

    QString qChange = ui->lineEdit_1Col->text();
    std::string sChange = qChange.toStdString();
    ui->treeWidget->currentItem()->setText(0, qChange);

    // WARNING if there are two identical names it will change both
    for(int i = 0; i < mExtractDataArm.size(); i++){
        if(mExtractDataArm[i].first ==  sOld){
            mExtractDataArm[i].first =  sChange;
        }
    } 
    for(int i = 0; i < mExtractDataUAV.size(); i++){
        if(mExtractDataUAV[i].first ==  sOld){
            mExtractDataUAV[i].first =  sChange;
        }
    }
    for(int i = 0; i < mExtractDataPointCloud.size(); i++){
        if(mExtractDataPointCloud[i].first ==  sOld){
            mExtractDataPointCloud[i].first =  sChange;
        }
    }
    for(int i = 0; i < mExtractDataCamera.size(); i++){
        if(mExtractDataCamera[i].first ==  sOld){
            mExtractDataCamera[i].first =  sChange;
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::change2ColClicked()
{
    QBrush brush_green(Qt::green);
    ui->treeWidget->currentItem()->setBackground(1, brush_green);

    QString qOld = ui->treeWidget->currentItem()->text(1);
    std::string sOld = qOld.toStdString();

    QString qChange = ui->lineEdit_2Col->text();
    std::string sChange = qChange.toStdString();
    ui->treeWidget->currentItem()->setText(1, qChange);
    
    // WARNING if there are two identical names it will change both
    for(int i = 0; i < mExtractDataArm.size(); i++){
        if(mExtractDataArm[i].second ==  sOld){
            mExtractDataArm[i].second =  sChange;
        }
    } 
    for(int i = 0; i < mExtractDataUAV.size(); i++){
        if(mExtractDataUAV[i].second ==  sOld){
            mExtractDataUAV[i].second =  sChange;
        }
    }
    for(int i = 0; i < mExtractDataPointCloud.size(); i++){
        if(mExtractDataPointCloud[i].second ==  sOld){
            mExtractDataPointCloud[i].second =  sChange;
        }
    }
    for(int i = 0; i < mExtractDataCamera.size(); i++){
        if(mExtractDataCamera[i].second ==  sOld){
            mExtractDataCamera[i].second =  sChange;
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::execWindows()
{
    std::cout << "Exec Windows" << std::endl;
    this->close();
    // TODO: HACER GENERAL!!! SE PUEDA METER TODOS LOS UAV Y ARMS QUE SE QUIERA!!
    // TODO: HACER QUE SE PUEDA CAMBIAR MANUALMENTE!!

    //uav_gui = new UAV_gui(this);
    //uav_gui->configureGUI(mExtractDataUAV, mArgParser);
    //uav_gui->show();

    arm_gui = new Arm_gui(this);
    arm_gui->configureGUI(mExtractDataArm);
    arm_gui->show();

    //pclviewer_gui = new PCLViewer_gui(this);
    //pclviewer_gui->configureGUI(mExtractDataPointCloud);
    //pclviewer_gui->show();
  
    //camera_gui = new Camera_gui(this);
    //camera_gui->configureGUI(mExtractDataCamera);
    //camera_gui->show();

}

//---------------------------------------------------------------------------------------------------------------------
void Gui::addRootGUI(QTreeWidgetItem *_item, QString _name, QString _description)
{   
    _item->setText(0, _name);
    _item->setText(1, _description);
    //ui->treeWidget->addTopLevelItem(_item);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::addChildGUI(QTreeWidgetItem *_parent, QString _name, QString _description)
{
    QTreeWidgetItem *item = new QTreeWidgetItem();
    item->setText(0, _name);
    item->setText(1, _description);
    _parent->addChild(item);
}

//---------------------------------------------------------------------------------------------------------------------
// Static Variable
grvc::utils::ArgumentParser *Gui::mArgParser = nullptr;