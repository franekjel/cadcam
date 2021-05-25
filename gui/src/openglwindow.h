#pragma once

#include <QMatrix4x4>
#include <QOpenGLFramebufferObject>
#include <QOpenGLFunctions_4_2_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLWindow>

#include "objects/object.h"

class OpenGLWindow : public QOpenGLWindow, public QOpenGLFunctions_4_2_Core {
    Q_OBJECT
public:
    explicit OpenGLWindow(const std::vector<std::pair<QVector3D, QVector3D>>& _points, const std::vector<std::pair<QMatrix4x4, QMatrix4x4>>& _matrices, QWindow* parent = 0);
    ~OpenGLWindow();

private:
    QOpenGLFunctions_4_2_Core* f;

    std::vector<std::pair<QVector3D, QVector3D>> points;
    std::vector<std::pair<QMatrix4x4, QMatrix4x4>> matrices;

    std::vector<std::shared_ptr<Point>> points_;
    std::unique_ptr<Object> grid;
    std::shared_ptr<Cube> cube;

    int cur_trans = 0;

    QOpenGLShaderProgram program;
    GLuint u_proj, u_view, u_trans, u_color, u_grid;

    QMatrix4x4 m_proj, m_view;

    Camera camera;

    void updateCamera();
    void rotateAroundPoint(std::shared_ptr<Object> object, const QVector3D& point, const QVector3D& axis, const float& angle);

    bool draggedRight;
    QPoint lastMousePoint = QPoint(-1, -1);

    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mouseMoveEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
};
