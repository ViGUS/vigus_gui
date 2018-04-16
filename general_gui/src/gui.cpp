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

    ui->treeWidget->setColumnCount(2);
    ui->treeWidget->setHeaderLabels(QStringList() << "Windows" << "Data");

    }

//---------------------------------------------------------------------------------------------------------------------
Gui::~Gui()
{
    delete ui;
}

//---------------------------------------------------------------------------------------------------------------------
bool Gui::extractData(std::string _pathXML)
{
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
            addRoot(mItemRootUAV, "UAV", QString::number(contUAV));

            tinyxml2::XMLElement* itemUAV = childUAV->FirstChildElement("idUAV");
            int idUAV;
            resultXML = itemUAV->QueryIntText(&idUAV);
            if(resultXML != tinyxml2::XML_SUCCESS){
                std::cout << "Error extracting id UAV" << std::endl;   
            }
            else{
                // Add Child for UAV Root
                addChild(mItemRootUAV, "id UAV", QString::number(idUAV));
            }

            // TODO: guardar en vector datos de UAV para luego iniciar la gui con esos datos
               
        }

        // READ DATA FROM ARM IF EXIST 
        int contArm = 0;
        for (tinyxml2::XMLElement* childArm = rootWindows->FirstChildElement("Arm"); childArm != NULL; childArm = childArm->NextSiblingElement("Arm")){
            contArm++;

            // Add Root to GUI
            mItemRootArm = new QTreeWidgetItem(ui->treeWidget);
            addRoot(mItemRootArm, "Arm", QString::number(contArm));

            tinyxml2::XMLElement* itemArm = childArm->FirstChildElement("Backend");
            std::string backend;
            backend = itemArm->GetText();
            // Add Child for Arm Root
            addChild(mItemRootArm, "Backend", QString::fromStdString(backend));
            mExtractDataArm.push_back(std::make_pair("Backend", backend));

            itemArm = childArm->FirstChildElement("SerialPort");
            std::string serialPort;
            serialPort = itemArm->GetText();
            // Add Child for Arm Root
            addChild(mItemRootArm, "Serial Port", QString::fromStdString(serialPort));
            mExtractDataArm.push_back(std::make_pair("Serial Port", serialPort));

        }



    }

    return true;

}

//---------------------------------------------------------------------------------------------------------------------
void Gui::okeyClicked()
{
    std::cout << "Thats okey for you, continue!" << std::endl;
    execWindows();
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::execWindows()
{
    std::cout << "Exec Windows" << std::endl;
    this->close();

    arm_gui = new Arm_gui(this);
    arm_gui->configureGUI(mExtractDataArm);
    arm_gui->show();
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::addRoot(QTreeWidgetItem *_item, QString _name, QString _description)
{   
    _item->setText(0, _name);
    _item->setText(1, _description);
    //ui->treeWidget->addTopLevelItem(_item);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::addChild(QTreeWidgetItem *_parent, QString _name, QString _description)
{
    QTreeWidgetItem *item = new QTreeWidgetItem();
    item->setText(0, _name);
    item->setText(1, _description);
    _parent->addChild(item);
}