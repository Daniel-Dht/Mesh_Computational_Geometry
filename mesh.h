#ifndef MESH_H
#define MESH_H

#include <string>
#include <cmath>
#include <QGLWidget>
#include <iostream>
using namespace std;

struct Edge {
   int id_face;
   int id_adj;
};

class Point
{


public:    
    double _x;
    double _y;
    double _z;

    double value=0;
    int _face = -1;
    Point():_x(),_y(),_z() {}
    Point(float x_, float y_, float z_):_x(x_),_y(y_),_z(z_) {}

    // get
    double x() const { return _x; }
    double y() const { return _y; }
    double z() const { return _z; }
    int face() const { return _face; }

    double dot(const Point &p) const { return _x*p._x + _y*p._y + _z*p._z;}
    double getNorm() const { return std::sqrt(_x*_x + _y*_y + _z*_z) ;}

    Point operator*(double a) const { return Point(_x*a, _y*a, _z*a);}
    Point operator/(double a) const { return Point(_x/a, _y/a, _z/a);}
    Point operator+(const Point &p) const { return Point(_x+p._x, _y+p._y, _z+p._z);}
    Point operator-(const Point &p) const { return Point(_x-p._x, _y-p._y, _z-p._z);}    
    Point cross(const Point &p) const {return Point(_y*p._z - _z*p._y,       _z*p._x - _x*p._z,       _x*p._y - _y*p._x );};
    Point getNormalized() const {double a = std::sqrt(_x*_x + _y*_y + _z*_z); return Point(_x/a, _y/a, _z/a);}
    double operator[](int i) const {
        if(i==0) return _x;
        else if(i==1) return _y;
        else return _z;
    }

    friend ostream& operator<<(ostream& os, const Point& dt);

};
typedef Point Vector3d;

class Face
{
public:

    int _v[3] = {-1,-1,-1};
    int _adj[3] = {-1,-1,-1};

    Vector3d normal;
    Face(){};
    Face(int v1, int v2, int v3) {
        _v[0]=v1;_v[1]=v2;_v[2]=v3;
    }
    // get
    double v1() const { return _v[0]; }
    double v2() const { return _v[1]; }
    double v3() const { return _v[2]; }
    int adj1() const { return _adj[0]; }
    int adj2() const { return _adj[1]; }
    int adj3() const { return _adj[2]; }    


    friend ostream& operator<<(ostream& os, const Face& dt);
};

class Circulator;

class Mesh
{

public:
    int Nv, Nf;
    QVector<Point> vertexTab;
    QVector<Point> circumFacesCenters;
    QVector<Face> faces;
    QVector<Vector3d> laplacien;
    int DISPLAY_MODE = 0;
    bool DRAW_LAPLACIAN=false;
    bool DRAW_NORMAL_FACE=false;
    double NORMAL_SCALE=1;
    bool MODE_ONLY_2D = false;
    Mesh();
    
    void drawMesh() const;
    void drawMeshWireFrame() const;
    void drawVertices() const;
    void drawLaplacian() const;
    void drawNormalFace() const;
    void drawVoronoi() const;
    void parse(const char file_name[]);
    void setAdjFaces();
    void computeLaplacian();
    void computeFacesNormal();
    bool pointInTriangle(const Point &P, const int &i_triangle)  const;
    bool pointInCircumCircle(Point P, const int &i_triangle) const;
    bool edgeIsDelaunay(const Edge &e) const;

    void flipEdge(int id_face, int id_adj, std::vector<Edge> &edges_to_test);
    void splitTriangle(Point newVert, int idf, std::vector<Edge> &edges_to_test);
    void addRandomVertex();

    void setCircumCenter(int id_face, Point &center) const;
    void computeVoronoi();

    Point getCenterofFace(int face_id) const;
    Circulator circulator_begin(int i);
    Circulator circulator_end(int i);
};

class Circulator
{
    /* Iterator allowing to turn around a vertex, and getting its incident faces reference
     * It can be used like this:
     *
     * for(Circulator it=circulator_begin(id_vertex) ; it!=circulator_end(id_vertex); ++it){
     *     Face &f = *it;
     *     cout << f.v1() << ", " << f.v2() << ", " << f.v3() << ", " << endl;
     *     // or:
     *     cout << it.v_id << ", " << it.va << ", " << it.vb << ", " << endl;
     *
     * va and vb attributes are the opposed vertices indices of the vertex we are turning around,
     * in trigonometric order: v_id, va , vb
     *
     * To turn around and only treat the opposed vertices, use it.vb
    */

public:
    Mesh * mesh;
    int v_id;
    int id_first_face;
    int id_current_face;
    int id_next_face;
    int count=0;
    bool stop=false;
    int va, vb;

    Circulator(Mesh *mesh_, int v_id_, bool stop_=false){
        mesh = mesh_;
        v_id = v_id_;
        id_first_face = mesh->vertexTab[v_id].face();
        id_current_face = id_first_face;
        stop = stop_;
        compute_next_face_id();
    }

    void compute_next_face_id(){

        const Face & curr_face = mesh->faces[id_current_face];
        if      (curr_face.v1()==v_id) {id_next_face = curr_face.adj2();va=curr_face.v2(); vb=curr_face.v3();}
        else if (curr_face.v2()==v_id) {id_next_face = curr_face.adj3();va=curr_face.v3(); vb=curr_face.v1();}
        else                           {id_next_face = curr_face.adj1();va=curr_face.v1(); vb=curr_face.v2();}
    }

    Face& operator*(){
        return mesh->faces[id_current_face]; //(*it).adj
    }

    Circulator& operator++(){
        id_current_face = id_next_face;
        if(id_current_face==id_first_face) {
            stop = true;
        }
        compute_next_face_id();

        count++;
        return *this;
    }

};


#endif // MESH_H

