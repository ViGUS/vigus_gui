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
    tinyxml2::XMLError resultXML = xml_doc.LoadFile(_pathXML.c_str());
    if(resultXML != tinyxml2::XML_SUCCESS){ 
        std::cout << "Error loading file" << std::endl; 
        std::cout << resultXML << std::endl; 
        return false;
    }

    tinyxml2::XMLNode* rootWindows = xml_doc.FirstChildElement("Windows");
    if(rootWindows == nullptr){
        std::cout << "Error Windows root" << std::endl; 
        return false;
    }

    // READ DATA FROM UAV
    tinyxml2::XMLElement* elementQtyUAV = rootWindows->FirstChildElement("quantityUAV");
    if(elementQtyUAV == nullptr){
        std::cout << "Error element quantityUAV" << std::endl; 
        return false;
    }
    int numUAV;
    resultXML = elementQtyUAV->QueryIntText(&numUAV);
    if(resultXML != tinyxml2::XML_SUCCESS){ 
        std::cout << "Error extracting quantityUAV" << std::endl; 
        return false;
    }

    tinyxml2::XMLElement* elementUAV = rootWindows->FirstChildElement("UAV");
    if(elementUAV == nullptr){
        std::cout << "Error element UAV" << std::endl; 
        return false;
    }
    itemRootUAV = new QTreeWidgetItem(ui->treeWidget);
    addRoot(itemRootUAV, "UAV", "1");
    int i = 0;
    while(elementUAV != nullptr && i < numUAV){

        tinyxml2::XMLElement* itemUAV = elementUAV->FirstChildElement("idUAV");
        int idUAV;
        resultXML = itemUAV->QueryIntText(&idUAV);
        if(resultXML != tinyxml2::XML_SUCCESS){ 
            std::cout << "Error extracting id UAV" << std::endl; 
            return false;
        }
        addChild(itemRootUAV, "id UAV", QString::number(idUAV));

        // TODO: HACER COSA CON LOS DATOS QUE EXTRAIGAMOS DEL UAV

        i++;
    }

    // READ DATA FROM ARM 
    tinyxml2::XMLElement* elementQtyArm = rootWindows->FirstChildElement("quantityArm");
    if(elementQtyUAV == nullptr){
        std::cout << "Error element quantityArm" << std::endl; 
        return false;
    }
    int numArm;
    resultXML = elementQtyArm->QueryIntText(&numArm);
    if(resultXML != tinyxml2::XML_SUCCESS){ 
        std::cout << "Error extracting quantityArm" << std::endl; 
        return false;
    }

    tinyxml2::XMLElement* elementArm = rootWindows->FirstChildElement("Arm");
    if(elementArm == nullptr){
        std::cout << "Error element Arm" << std::endl; 
        return false;
    }
    itemRootArm = new QTreeWidgetItem(ui->treeWidget);
    addRoot(itemRootArm, "Arm", "1");
    int j = 0;
    while(elementArm != nullptr && j < numArm){

        tinyxml2::XMLElement* itemArm = elementArm->FirstChildElement("idArm");
        int idArm;
        resultXML = itemArm->QueryIntText(&idArm);
        if(resultXML != tinyxml2::XML_SUCCESS){ 
            std::cout << "Error extracting id Arm" << std::endl; 
            return false;
        }
        addChild(itemRootArm, "id Arm", QString::number(idArm));

        // TODO: HACER COSA CON LOS DATOS QUE EXTRAIGAMOS DE LOS BRAZOS

        j++;
    }



}

//---------------------------------------------------------------------------------------------------------------------
void Gui::okeyClicked()
{

    std::cout << "Te parece todo bien, sigamos" << std::endl;
    execWindows();
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::execWindows()
{

    std::cout << "Se ejecutaran las ventanas extraidas del xml" << std::endl;

}

//---------------------------------------------------------------------------------------------------------------------
void Gui::addRoot(QTreeWidgetItem *_item, QString _name, QString _description)
{
    _item->setText(0, _name);
    _item->setText(1, _description);
    //ui->treeWidget->addTopLevelItem(item);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::addChild(QTreeWidgetItem *_parent, QString _name, QString _description)
{
    QTreeWidgetItem *item = new QTreeWidgetItem();
    item->setText(0, _name);
    item->setText(1, _description);
    _parent->addChild(item);
}