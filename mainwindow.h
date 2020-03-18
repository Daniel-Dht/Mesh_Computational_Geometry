#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_RButton_vertices_only_clicked();
    void on_RButton_plain_face_clicked();
    void on_RButton_wire_face_clicked();

    void on_PButton_drawLaplacian_clicked();
    void on_PButton_draw_FaceNormal_clicked();

    void on_Slider_normalScaling_valueChanged(int value);

    void on_PButton_loadFile_clicked();

    void on_PButton_FlipEdge_clicked();

    void on_PButton_computeLap_clicked();

    void on_PButton_ResetVertVal_clicked();

    void on_PButttun_addVert_clicked();

    void on_RB_Voronoi_clicked();

    void on_CB_2D_only_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
