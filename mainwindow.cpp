#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "mesh.h"

#include<iostream>
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //connect(ui->RButton_vertices_only, SIGNAL(clicked()), this, SLOT(on_RButton_vertices_only_clicked()));
    //connect(ui->RButton_plain_face, SIGNAL(clicked()), this, SLOT(on_RButton_plain_face_clicked()));    
    on_Slider_normalScaling_valueChanged(10);
    //connect(ui->PButton_FlipEdge, SIGNAL(clicked()), this, SLOT(ui->on_PButton_FlipEdge_clicked()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_RButton_vertices_only_clicked() { ui->widget->_mesh.DISPLAY_MODE = 0;}
void MainWindow::on_RButton_plain_face_clicked()    { ui->widget->_mesh.DISPLAY_MODE = 1;}
void MainWindow::on_RButton_wire_face_clicked()     { ui->widget->_mesh.DISPLAY_MODE = 2;}
void MainWindow::on_RB_Voronoi_clicked(){
    ui->widget->_mesh.computeVoronoi();
    ui->widget->_mesh.DISPLAY_MODE = 3;
}

void MainWindow::on_PButton_drawLaplacian_clicked()  { ui->widget->_mesh.DRAW_LAPLACIAN ^= true;}
void MainWindow::on_PButton_draw_FaceNormal_clicked(){ ui->widget->_mesh.DRAW_NORMAL_FACE ^= true;}

void MainWindow::on_Slider_normalScaling_valueChanged(int value) {ui->widget->_mesh.NORMAL_SCALE = double(value)/100;}

void MainWindow::on_PButton_loadFile_clicked()
{
    QString s_file("../Mesh_Computational_Geometry/"+ui->CBox_fileNames->currentText()+".off");
    const char* c_file= s_file.toStdString().c_str();
    ui->widget->_mesh.parse(c_file);
}

void MainWindow::on_PButton_FlipEdge_clicked() {
    std::cout << "flipping edge" << std::endl;
    //ui->widget->_mesh.flipEdge(0, 1) ;
    std::cout << "flipping edge" << std::endl;
//    ui->widget->_mesh.flipEdge(0, 0) ;
//    std::cout << ".... end" << std::endl;
//    ui->widget->_mesh.flipEdge(0, 2) ;
//    std::cout << ".... end" << std::endl;
//    ui->widget->_mesh.flipEdge(0, 1) ;
//    std::cout << ".... end" << std::endl;
}

void MainWindow::on_PButton_computeLap_clicked()
{
    ui->widget->_mesh.setAdjFaces();
    ui->widget->_mesh.computeLaplacian();
}

void MainWindow::on_PButton_ResetVertVal_clicked()
{
    for (int i=0 ; i < ui->widget->_mesh.Nv; i++){
        ui->widget->_mesh.vertexTab[i].value = 0;
    }
}

void MainWindow::on_PButttun_addVert_clicked()
{
    //ui->widget->_mesh.triangleSplit(0, Point(-0.5, 0, 0));
    ui->widget->_mesh.addRandomVertex();
}

void MainWindow::on_CB_2D_only_clicked() { ui->widget->_mesh.MODE_ONLY_2D ^= true;}
