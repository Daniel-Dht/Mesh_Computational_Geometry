#include "mesh.h"
#include <GL/glu.h>

// Draw a vertex
void glVertexDraw(const Point & p) {
    glVertex3f(p.x(), p.y(), p.z());
}


//Example with a tetraedra
void Mesh::drawMesh() const {
    for(int i = 0; i < Nf; i++) {

        const Face &face = faces[i];
        glColor3f(0, 1 ,0);

        glBegin(GL_TRIANGLES);
            glVertexDraw(vertexTab[face.v1()]);
            glVertexDraw(vertexTab[face.v2()]);
            glVertexDraw(vertexTab[face.v3()]);
        glEnd();
    }
}

//Example with a wireframe tedraedra
void Mesh::drawMeshWireFrame() const {
    for(int i = 0; i < Nf; i++) {

        const Face &face = faces[i];
        if (MODE_ONLY_2D && vertexTab[face.v3()].z() !=0 ) continue;
        glColor3f(0, 1 ,0);
        glBegin(GL_LINE_STRIP);
            glVertexDraw(vertexTab[face.v1()]);
            glVertexDraw(vertexTab[face.v2()]);
        glEnd();
        glBegin(GL_LINE_STRIP);
            glVertexDraw(vertexTab[face.v2()]);
            glVertexDraw(vertexTab[face.v3()]);
        glEnd();
        glBegin(GL_LINE_STRIP);
            glVertexDraw(vertexTab[face.v3()]);
            glVertexDraw(vertexTab[face.v1()]);
        glEnd();
    }
}


void Mesh::drawVertices() const {

    for(int i = 0; i < Nv; i++) {

        const double &value = vertexTab[i].value;
        if(value < 0) glColor3d(-value,0,0);
        if(value > 0) glColor3d(0,value,0);
        // else glColor3d(1,1,0);

        glPointSize(0.5);
        glBegin(GL_POINTS);
            glVertexDraw(vertexTab[i]);
        glEnd();
    }
}

void Mesh::drawLaplacian() const {
    for(int i = 0; i < Nv; i++) {

        const Point &p1 = vertexTab[i];
        const Point &dir = laplacien[i];

        glColor3f(1, 0 ,0);
        glBegin(GL_LINE_STRIP);
            glVertexDraw(p1);
            glVertexDraw(p1 + dir*NORMAL_SCALE);
        glEnd();
    }
}
void Mesh::drawNormalFace() const {

    for(int i = 0; i < Nf; i++) {

        const Point p1 = getCenterofFace(i);
        const Point &dir = faces[i].normal;

        glColor3d(1,1,0);
        glBegin(GL_LINE_STRIP);
            glVertexDraw(p1);
            glVertexDraw(p1 + dir*NORMAL_SCALE);
        glEnd();
    }
}

void Mesh::drawVoronoi() const {

    glColor3d(1,1,0);
    for(int i = 0; i < Nv; i++) {
        if (vertexTab[i].z() != 0) continue;
        glPointSize(3.);
        glBegin(GL_POINTS);
            glVertexDraw(vertexTab[i]);
        glEnd();
    }

    for(int i = 0; i < Nf; i++) {

        const Face &face = faces[i];
        if (vertexTab[face.v3()].z() !=0 ||
            vertexTab[face.v2()].z() !=0 ||
            vertexTab[face.v1()].z() !=0) continue;

        const Point &cicumcenter = circumFacesCenters[i];

        glColor3f(0, 1 ,0);

        const Face &adj1 = faces[face.adj1()];
        if (vertexTab[adj1.v3()].z() ==0 &&
            vertexTab[adj1.v2()].z() ==0 &&
            vertexTab[adj1.v1()].z() ==0) {
            glBegin(GL_LINE_STRIP);
                glVertexDraw(cicumcenter);
                glVertexDraw(circumFacesCenters[face.adj1()]);
            glEnd();
        }
        const Face &adj2 = faces[face.adj2()];
        if (vertexTab[adj2.v3()].z() ==0 &&
            vertexTab[adj2.v2()].z() ==0 &&
            vertexTab[adj2.v1()].z() ==0) {
            glBegin(GL_LINE_STRIP);
                glVertexDraw(cicumcenter);
                glVertexDraw(circumFacesCenters[face.adj2()]);
            glEnd();
         }
        const Face &adj3 = faces[face.adj3()];
        if (vertexTab[adj3.v3()].z() ==0 &&
            vertexTab[adj3.v2()].z() ==0 &&
            vertexTab[adj3.v1()].z() ==0) {
            glBegin(GL_LINE_STRIP);
                glVertexDraw(cicumcenter);
                glVertexDraw(circumFacesCenters[face.adj3()]);
            glEnd();
        }

    }
}

