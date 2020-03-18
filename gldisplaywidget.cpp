#include "gldisplaywidget.h"
#ifdef __APPLE__
    #include <glu.h>
#else
    #include <GL/glu.h>
#endif
#include <iostream>
#include "QDebug"

GLDisplayWidget::GLDisplayWidget(QWidget *parent) : QGLWidget(parent)
{
    // Update the scene
    connect( &_timer, SIGNAL(timeout()), this, SLOT(updateGL()));
    _timer.start(16);
}

void GLDisplayWidget::initializeGL()
{
    // background color
    glClearColor(0.2, 0.2, 0.2, 1);

    // Shader
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);

    std::cout << "graphic card: "<< glGetString(GL_RENDERER) << std::endl;
}

void GLDisplayWidget::paintGL(){

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Center the camera
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0,0,5,  0,0,0,   0,1,0);

    // Translation
    glTranslated(_X, _Y, _Z);

    // Rotation
    glRotatef(_angleX, 1.0f, 1.0f, 0.0f);
    glRotatef(1.0f, _angleY, 1.0f, 0.0f);

    // Color for your mesh
    glColor3f(0, 1 ,0);

    // example with a tetraedre
    if      (_mesh.DISPLAY_MODE==0){ _mesh.drawVertices();}
    else if (_mesh.DISPLAY_MODE==1){ _mesh.drawMesh();}
    else if (_mesh.DISPLAY_MODE==2){ _mesh.drawMeshWireFrame();}
    else if (_mesh.DISPLAY_MODE==3){ _mesh.drawVoronoi();}

    if(_mesh.DRAW_LAPLACIAN) _mesh.drawLaplacian();
    if(_mesh.DRAW_NORMAL_FACE) _mesh.drawNormalFace();

//    Circulator it = circulator_begin(i);
//    while (!it.stop){

//    }
}

void GLDisplayWidget::resizeGL(int width, int height){

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (GLfloat)width/(GLfloat)height, 0.1f, 100.0f);

    updateGL();
}

// - - - - - - - - - - - - Mouse Management  - - - - - - - - - - - - - - - -
// When you click, the position of your mouse is saved
void GLDisplayWidget::mousePressEvent(QMouseEvent *event)
{
    if( event != NULL )
        _lastPosMouse = event->pos();
}

// Mouse movement management
void GLDisplayWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - _lastPosMouse.x();
    int dy = event->y() - _lastPosMouse.y();

    if( event != NULL )
    {
        _angleX += dx;
        _angleY += dy;
        _lastPosMouse = event->pos();

        updateGL();
    }
}

// Mouse Management for the zoom
void GLDisplayWidget::wheelEvent(QWheelEvent *event) {
    QPoint numDegrees = event->angleDelta();
    double stepZoom = 0.2;
    if (!numDegrees.isNull())
    {
      _Z = (numDegrees.x() > 0 || numDegrees.y() > 0) ? _Z + stepZoom : _Z - stepZoom;
    }
}
