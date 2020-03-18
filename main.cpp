#include "mainwindow.h"
#include <QApplication>

// C:\Users\Daniel.D\Documents\Centrale Pédagogie\3A\MSO\info_graphique\Mesh_Computational_Geometry\Mesh_Computational_Geometry.pro

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}


//	Structure de données + chargement OFF
//Circulateurs + Courbures

//	Triangulation naïve avec triangles coupés en 3


//	Algorithme de Lawson global pour transformer une triangulation naïve en triangulation de Delaunay (avec Flip d’arête)
//Prédicat être localement de Delaunay pour une arête – ou utilisation des angles pi – (alpha + beta)

//   OU

//	(Possibilité d’ajouter un point dans une triangulation de Delaunay en ne faisant que le nombre de test de
//   flips nécessaires. On fait toute la triangulation en restant de Delaunay.)

//	Marche pour localiser le triangle où on insère un point

//	Affichage de Voronoï ou CRUST
