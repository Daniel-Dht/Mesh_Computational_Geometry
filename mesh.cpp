#include "mesh.h"

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>

#include<map>

#include <random>
std::random_device rd;
std::default_random_engine engine(rd());
std::uniform_real_distribution<> uniform(-0.98, 0.98);


//#include <chrono>
//using namespace std::chrono;
//auto start = high_resolution_clock::now();
//auto stop = high_resolution_clock::now();
//auto duration = duration_cast<microseconds>(stop - start);
//cout << "Time taken for adjacent triangle : " << duration.count() << " seconds" << endl;



using namespace std;
Mesh::Mesh()
{
    parse("../Mesh_Computational_Geometry/test.off");
//    for (int i=0 ; i < Nf; i++)
//    {
//        Face &f = faces[i] ;
//        cout << f <<endl;

//    }

    //computeFacesNormal();
    //computeLaplacian();
    //cout << "isInTriangle " << isInTriangle() <<endl;

//    Point center = Point(5,5,5);
//    //setCircumCenter(4, center);

//    int id_face = 4;
//    Point A = vertexTab[faces[id_face]._v[0]];
//    Point B = vertexTab[faces[id_face]._v[1]];
//    Point C = vertexTab[faces[id_face]._v[2]];
//    tricircumcenter(A,B,C, center);
//    cout << "ACACACAC  " << center << endl;
}

Circulator Mesh::circulator_begin(int i){ return Circulator(this, i)   ;}
Circulator Mesh::circulator_end(  int i){ return Circulator(this, i, true);}

void Mesh::parse(const char file_name[])
{
    FILE *fp;
    fp = fopen(file_name, "r");  //Ouverture d'un fichier en lecture
    if(fp == NULL)
    {
        cout <<"Error opening file: "<< file_name << endl;
        //exit(1);
        return;
    }
    cout <<"Opening "<< file_name << endl;

    vertexTab.clear();
    laplacien.clear();
    faces.clear();
    int nb_edge = 0;
    fscanf(fp, "%d %d %d\n", &Nv, &Nf, &nb_edge);
    cout <<"nb vertices: "<< Nv<< ", nb faces: " << Nf  << endl;

    vertexTab.reserve(Nv);
    laplacien.reserve(Nv);
    faces.reserve(Nf);

    double x, y, z;
    for(int i_vertex = 0; i_vertex < Nv; i_vertex++)
    {
        fscanf(fp, "%lf %lf %lf\n", &x, &y, &z);
        vertexTab.push_back(Point(x,y,z));
        laplacien.push_back(Point(0, 0, 0));
    }

    int n_face, v1, v2, v3;
    for(int i_triangle = 0; i_triangle < Nf; i_triangle++)
    {
        fscanf(fp, "%d %d %d %d\n", &n_face, &v1, &v2, &v3);
        faces.push_back(Face(v1, v2, v3));
    }
    fclose(fp);

    setAdjFaces();
    //computeLaplacian();
}


std::pair<int, int> getOrderedPair(int a, int b){
    if(a<b) return {a, b};
    else    return {b, a};
}

void Mesh::setAdjFaces()
{
    // set also the face attribute of vertices
    bool vert_done[Nv];
    for(int i = 0; i < Nv; i++) vert_done[i] = false;

    std::map<std::pair<int,int>, std::pair<int,int>> myMap; // careful: Accessing a non-existing element with "[]" creates it
    for(int i = 0; i < Nf; i++) {

        Face * face = &faces[i];
        int v1 = face->v1();
        int v2 = face->v2();
        int v3 = face->v3();

        // setting incident face for vertices:
        if(!vert_done[v1]){ vert_done[v1]=true; vertexTab[v1]._face=i; };
        if(!vert_done[v2]){ vert_done[v2]=true; vertexTab[v2]._face=i; };
        if(!vert_done[v3]){ vert_done[v3]=true; vertexTab[v3]._face=i; };

        // setting adjacent faces on each edge:
        std::pair <int, int> p;      //  {vertex id n°1, vertex id n°2}
        std::pair <int, int> pfound; //  {face id whose edge was'nt yet register in the map,  its opposed vertex id (0,1 or 2) }

        p = getOrderedPair(v1, v2); //-> opposed vertex indice is 2
        if(myMap.find(p) == myMap.end()){
            myMap[p] = {i,2};
        } else{
            pfound = myMap.at(p);
            face->_adj[2] = pfound.first;
            // the face that originally didn't found its adjacent face on this parictular edge is now treated:
            faces[pfound.first]._adj[pfound.second] = i;
        }

        p = getOrderedPair(v1, v3);
        if(myMap.find(p) == myMap.end()){
            myMap[p] = {i,1};
        } else{
            pfound = myMap.at(p);
            face->_adj[1] = pfound.first;
            faces[pfound.first]._adj[pfound.second] = i;
        }

        p = getOrderedPair(v2, v3);
        if(myMap.find(p) == myMap.end()){
            myMap[p] = {i,0};
        } else{
            pfound = myMap.at(p);
            face->_adj[0] = pfound.first;
            faces[pfound.first]._adj[pfound.second] = i;
        }
    }
}

int getFaceIDinAdjTab(const Face &f, const int id_f){
    // En venant de la face adjacente de f d'id id_f;
    // return son id dans le tableau d'adjacence.
    for(int i=0; i<3 ; i++) {
        if (f._adj[i] == id_f){
            return i;
        }
    }
    exit(4);
}

double computeCotan(Vector3d &A, Vector3d &B)
{
    double x = A.dot(B)/(A.getNorm()*B.getNorm());
    return x/sqrt(1-x*x);
}

void Mesh::computeFacesNormal()
{
    for (int i=0 ; i < Nf; i++)
    {
        Face &f = faces[i];

        Vector3d A = vertexTab[f.v2()]-vertexTab[f.v1()];
        Vector3d B = vertexTab[f.v3()]-vertexTab[f.v1()];
        f.normal = A.cross(B).getNormalized();
    }
}

void Mesh::computeLaplacian()
{    
    cout <<"Computing Laplacian..."<< Nv << endl;
    computeFacesNormal();

    laplacien.clear();
    laplacien.reserve(Nv);
    // keep min and max to normalize at the end
    double min_lap = 1E99;
    double max_lap = 0;

    for (int i=0 ; i < Nv; i++)
    {
        laplacien.push_back(Vector3d(0,0,0));
        double area = 0;
        Circulator it = circulator_begin(i);
        while (!it.stop){

            Vector3d diff   = vertexTab[it.vb]-vertexTab[i];
            Vector3d vectA1 = vertexTab[i]    -vertexTab[it.va];
            Vector3d vectA2 = vertexTab[it.vb]-vertexTab[it.va];
            area += vectA1.cross(vectA2).getNorm()/2 /3; // divided by 3 allows the approximation of barycentric area around the v_id vertex
            ++it;

            Vector3d vectB1 = vertexTab[i]    -vertexTab[it.vb];
            Vector3d vectB2 = vertexTab[it.va]-vertexTab[it.vb];
            double cotanA = computeCotan(vectA1, vectA2);
            double cotanB = computeCotan(vectB1, vectB2);

            laplacien[i] = laplacien[i] + diff*(cotanA + cotanB);
        }

        laplacien[i] = laplacien[i]/(2*area);

        // check sign direction of laplacian with the normal of one of the triangle
        Face &f = *it;

        // assing the laplacian norm with the right sign to the vertex value
        double signe = 1.0;

        f.normal.dot(laplacien[i]) > 0 ? signe=1.0 : signe = -1.0;

        double lapNorme = laplacien[i].getNorm();
        laplacien[i] = laplacien[i]/lapNorme * signe;

        lapNorme = std::pow(lapNorme, 0.1); // better constrast between convex and concav small values

        double &value = vertexTab[i].value;
        value = signe * lapNorme;
        if(value < min_lap) min_lap = value;
        if(value > max_lap) max_lap = value;
    }
    // normalise vertex value between -1 and 1
    for (int i=0 ; i < Nv; i++){
        double &value = vertexTab[i].value;
        if(value <0) value /= -1*min_lap; // * -1 to keep sign negativ
        if(value >0) value /= max_lap;
    }

    cout <<"Laplacian computed !" << endl;
}

Point Mesh::getCenterofFace(int face_id) const{
    const Face &f = faces[face_id];
    const Point p1 = vertexTab[f.v1()];
    const Point p2 = vertexTab[f.v2()];
    const Point p3 = vertexTab[f.v3()];
    return (p1+p2+p3)/3;
}

ostream& operator<<(ostream& os, const Point& V)
{
    os << V.x() << ",  " << V.y() << ",  " << V.z();
    return os;
}

ostream& operator<<(ostream& os, const Face& f)
{
    os << f.v1() << ",  " << f.v2() << ",  " << f.v3() << endl;
    os << f.adj1() << ",  " << f.adj2() << ",  " << f.adj3() << endl;
    return os;
}

void Mesh::flipEdge(const int id_f1, const int id_adjf1, std::vector<Edge> &edges_to_test){

   Face &f1 = faces[id_f1];

   const int id_f2 = f1._adj[id_adjf1];
   Face &f2 = faces[id_f2];

   // vertex opposé au coté adjaçant
   const int f1_v_opp = id_adjf1;
   const int f2_v_opp = getFaceIDinAdjTab(f2, id_f1); // cherche l'id de f1 dans le tableau d'adjacence de f2

   cout << "flipping edge !" << endl;
   // id vertex qui vont être modifiés
   const int f1_v_toBeUpdate = (f1_v_opp+1)%3;
   const int f2_v_toBeUpdate = (f2_v_opp+1)%3;

   // update vertices
   // si son attribut _face était set sur fi, on le change
   const int potential_v_f1 = f1._v[f1_v_toBeUpdate];
   const int potential_v_f2 = f2._v[f2_v_toBeUpdate];
   if (vertexTab[potential_v_f1]._face == id_f1){
       vertexTab[potential_v_f1]._face  = id_f2;
   }
   if (vertexTab[potential_v_f2]._face == id_f2){
       vertexTab[potential_v_f2]._face  = id_f1;
   }
   // update the vertices
   f1._v[f1_v_toBeUpdate] = f2._v[f2_v_opp];
   f2._v[f2_v_toBeUpdate] = f1._v[f1_v_opp];

   // id adj qui vont être modifiés
   const int f1_vopp_prev = (f1_v_opp+2)%3;
   const int f2_vopp_prev = (f2_v_opp+2)%3;

   // première update ajacence des 2 faces qui reste dans le tableau mais change de place:
   // f1_vopp_next = f1_v_toBeUpdate
   const int new_f2_adj_faceID = f1._adj[f1_vopp_prev]; // new adj face for f2
   const int new_f1_adj_faceID = f2._adj[f2_vopp_prev]; // new adj face for f1

   f1._adj[f1_v_opp] = new_f1_adj_faceID;
   f2._adj[f2_v_opp] = new_f2_adj_faceID;

   f1._adj[f1_vopp_prev] = id_f2;
   f2._adj[f2_vopp_prev] = id_f1;

    // update les tableaux d'adjacence des tableaux adjacents, il y en a seulement 2
    // cherche ou se trouvait f2 dans la nouvelle face adj de f1 pour y mettre f1, puis pareil pour f2
    Face &new_f1_adj_face = faces[new_f1_adj_faceID];
    Face &new_f2_adj_face = faces[new_f2_adj_faceID];
    int f1_id_in_its_new_adjFace = getFaceIDinAdjTab(new_f1_adj_face, id_f2);
    int f2_id_in_its_new_adjFace = getFaceIDinAdjTab(new_f2_adj_face, id_f1);
    new_f1_adj_face._adj[f1_id_in_its_new_adjFace] = id_f1;
    new_f2_adj_face._adj[f2_id_in_its_new_adjFace] = id_f2;

    // adding edges to be tested
    edges_to_test.push_back(Edge{id_f1, f1_v_toBeUpdate});
    edges_to_test.push_back(Edge{id_f1, f1_v_opp});
    edges_to_test.push_back(Edge{id_f2, f2_v_toBeUpdate});
    edges_to_test.push_back(Edge{id_f2, f2_v_opp});
}

bool orientationTest(const Point &A, const Point &B, const Point &C)
{
    return (B.x()-A.x())*(C.y()-A.y()) - (B.y()-A.y())*(C.x()-A.x()) > 0;
}

double orientationTest2D(const Point &A, const Point &B, const Point &C)
{
    return (B.x()-A.x())*(C.y()-A.y()) - (B.y()-A.y())*(C.x()-A.x()); // AB.cross(AC)
}

double orient2D(const Point &A, const Point &B, const Point &C)
{
    return (A.x()-C.x())*(B.y()-C.y()) - (A.y()-C.y())*(B.x()-C.x()); // AB.cross(AC)
}

bool Mesh::pointInTriangle(const Point &P, const int &id_face) const
{
    const Face &face = faces[id_face];
    if (vertexTab[face.v3()].z() !=0 ) return false;
    bool isIn = true;
    for (int k = 0; k < 3; ++k)
        isIn = isIn && orientationTest(vertexTab[face._v[k]], vertexTab[face._v[(k+1)%3]], P);
    return isIn;
}


bool Mesh::pointInCircumCircle(Point P, const int &id_face) const
{
    if (P.z() != 0)
        return false;
    Point A = vertexTab[faces[id_face]._v[0]];
    Point B = vertexTab[faces[id_face]._v[1]];
    Point C = vertexTab[faces[id_face]._v[2]];
    // utilise la coordonnée en z pour la taille des segments
    B._z = (B-A).dot(B-A); // ||AB||²
    C._z = (C-A).dot(C-A); // ||AC||²
    P._z = (P-A).dot(P-A); // ||AP||²
    return (B-A).cross(C-A).dot(P-A) < 0.;
}

void Mesh::splitTriangle(Point newVert, int idf, std::vector<Edge> &edges_to_test){
    // à partir d'un point dans une face, découpe cette face en 3
    // -> push le nouveau point dans vertexTab
    // -> modifie la face A
    // -> Créer 2 nouvelles faces : B et C
   // cout << "Nv " << Nv << endl;
    newVert._face = idf; // on lie le nouveau point à la face A
    vertexTab.push_back(newVert);
    laplacien.push_back(Vector3d(0,0,0));
    Nv++;

    // id des deux nouvelles faces
    int idB = Nf +0;
    int idC = Nf +1;

    // modification de la première face
    Face &f = faces[idf]; // f devient A
    vertexTab[f.v3()]._face = idB; // change vertex face attribute

    Face B = Face(f.v2(), f.v3(), Nv-1);
    Face C = Face(f.v3(), f.v1(), Nv-1);
    f._v[2] = Nv-1; //Face A = Face(f.v1(), f.v2(), Nv);

    // adj faces of initial f Face
    Face &adjf1 = faces[f.adj1()];
    Face &adjf2 = faces[f.adj2()];
    int f_id_in_adjf1 = getFaceIDinAdjTab(adjf1, idf);
    int f_id_in_adjf2 = getFaceIDinAdjTab(adjf2, idf);
    adjf1._adj[f_id_in_adjf1] = idB;
    adjf2._adj[f_id_in_adjf2] = idC;

    B._adj[0] = idC;
    B._adj[1] = idf;
    B._adj[2] = f.adj1();

    C._adj[0] = idf;
    C._adj[1] = idB;
    C._adj[2] = f.adj2();

    f._adj[0] = idB;
    f._adj[1] = idC;

    cout << "Vertex added ! " << endl;
    // compute normal of the new faces
    B.normal = (vertexTab[B.v2()]-vertexTab[B.v1()].cross(vertexTab[B.v3()]-vertexTab[B.v1()])).getNormalized();
    C.normal = (vertexTab[C.v2()]-vertexTab[C.v1()].cross(vertexTab[C.v3()]-vertexTab[C.v1()])).getNormalized();
    faces.push_back(B);
    faces.push_back(C);
    Nf += 2;

    // Add edges to be tested
    edges_to_test.push_back(Edge {idB, 2});
    edges_to_test.push_back(Edge {idC, 2});
    edges_to_test.push_back(Edge {idf, 2});
}

bool Mesh::edgeIsDelaunay(const Edge &e) const
{
    int id_adj_face = faces[e.id_face]._adj[e.id_adj];
    int edge_faceID_in_adj = 0;

    for (int k = 0; k < 3; ++k)
    {

        if (vertexTab[faces[id_adj_face]._v[k]].z() !=0 || vertexTab[faces[e.id_face]._v[k]].z() !=0) {
            cout << "doing Delaunay Test on 3D face..." << endl;
            return true;
        }
        if (faces[id_adj_face]._adj[k] == e.id_face)
            edge_faceID_in_adj = k;
    }
    int id_vertex = faces[id_adj_face]._v[edge_faceID_in_adj];
    return !pointInCircumCircle(vertexTab[id_vertex], e.id_face);
}

void Mesh::addRandomVertex() {

    double x = uniform(engine);
    double y = uniform(engine);
    //Point P = Point(0.75,0.75,0);
    Point P = Point(x,y,0);

    std::vector<Edge> edges_to_test;
    for(int i=0; i<Nf; i++){

        if( pointInTriangle(P, i)){
            splitTriangle(P, i, edges_to_test);
            break;
        }
    }

    int i =0;
    while (!edges_to_test.empty() && i<100)
    {
        i++;
        Edge e = edges_to_test.back();
        if ( !edgeIsDelaunay(e) ){
            flipEdge(e.id_face, e.id_adj, edges_to_test);
        }
        edges_to_test.pop_back();
    }

//    if (DISPLAY_MODE==3) { // (Voronoi display mode)
//        if (circumFacesCenters.size() == Nf){
//            Point circumcenter;
//            setCircumCenter(Nf, circumcenter);
//            circumFacesCenters.push_back(circumcenter);
//        }
//    }

}

void Mesh::setCircumCenter(int id_face, Point &circumcenter) const
{
  double xba, yba, xca, yca;
  double balength, calength;
  double denominator;

  Point a = vertexTab[faces[id_face]._v[0]];
  Point b = vertexTab[faces[id_face]._v[1]];
  Point c = vertexTab[faces[id_face]._v[2]];

  /* Use coordinates relative to point `a' of the triangle. */
  xba = b[0] - a[0];
  yba = b[1] - a[1];
  xca = c[0] - a[0];
  yca = c[1] - a[1];
  /* Squares of lengths of the edges incident to `a'. */
  balength = xba * xba + yba * yba;
  calength = xca * xca + yca * yca;

  /* Calculate the denominator of the formulae. */
  denominator = 0.5 / (xba * yca - yba * xca);

  /* Calculate offset (from `a') of circumcenter. */
  circumcenter._x = a._x + (yca * balength - yba * calength) * denominator;
  circumcenter._y = a._y + (xba * calength - xca * balength) * denominator;
  circumcenter._z = 0;
}

void Mesh::computeVoronoi() {
    circumFacesCenters.clear();
    circumFacesCenters.reserve(Nf);
    for(int i=0 ; i<Nf ; i++) {
        Point circumcenter;
        setCircumCenter(i, circumcenter);
        circumFacesCenters.push_back(circumcenter);
    }

}





















