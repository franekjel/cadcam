#pragma once
#include <QOpenGLWidget>

//we will use QOpenGLWidget
//docs with examples: https://doc.qt.io/qt-5/qopenglwidget.html
class OpenGLWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    explicit OpenGLWidget(QWidget* parent = nullptr);
    ~OpenGLWidget();

private:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
};
