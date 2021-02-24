#include "openglwidget.h"

OpenGLWidget::OpenGLWidget(QWidget* parent)
{
    auto f = format();
    //docs: https://doc.qt.io/qt-5/qsurfaceformat.html#QSurfaceFormat
    f.setRenderableType(QSurfaceFormat::OpenGL);
    f.setColorSpace(QSurfaceFormat::sRGBColorSpace);
    f.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    setFormat(f);
}

OpenGLWidget::~OpenGLWidget()
{
}

void OpenGLWidget::initializeGL()
{
}

void OpenGLWidget::resizeGL(int w, int h)
{
}

void OpenGLWidget::paintGL()
{
}
