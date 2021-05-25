#include "openglwindow.h"

#include <QMouseEvent>

#include <cstdlib>
#include <ctime>

OpenGLWindow::OpenGLWindow(const std::vector<std::pair<QVector3D, QVector3D>>& _points, const std::vector<std::pair<QMatrix4x4, QMatrix4x4>>& _matrices, QWindow* parent) {
    points = _points;
    matrices = _matrices;
    if (_matrices.empty())
        matrices.push_back({ Identity(), Identity() });

    camera.translation = QVector3D(0, 0, -6.0f);
    camera.RotateX(-0.8f);
    camera.RotateY(0.8f);

    m_view = camera.Matrix();
}

OpenGLWindow::~OpenGLWindow() {
    makeCurrent();
}

void OpenGLWindow::updateCamera() {
    makeCurrent();
    m_view = camera.Matrix();
    program.setUniformValue(u_view, m_view);
    update();
}

void OpenGLWindow::rotateAroundPoint(std::shared_ptr<Object> object, const QVector3D& point, const QVector3D& axis, const float& angle) {
    QQuaternion q = QQuaternion::fromAxisAndAngle(axis, angle);
    object->translation = q.rotatedVector((object->translation - point)) + point;
    object->rotation = q * object->rotation;
}

void OpenGLWindow::initializeGL() {
    initializeOpenGLFunctions();

    f = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_4_2_Core>();

    program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/cube_v.glsl");
    program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/cube_f.glsl");
    program.link();
    program.bind();
    u_view = program.uniformLocation("m_view");
    u_proj = program.uniformLocation("m_proj");
    u_trans = program.uniformLocation("m_trans");
    u_color = program.uniformLocation("m_color");
    u_grid = program.uniformLocation("grid");

    grid = std::unique_ptr<Object>(new Object({ 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, -1.0f, -1.0f, 0.0f, -1.0f, -1.0f, 0.0f, 1.0f }, { 0, 1, 3, 1, 2, 3 }, f));
    grid->scale(QVector3D(300.0f, 1.0f, 300.0f), QVector3D(0, 0, 0));
    grid->translation.setY(0.001f);

    points_.reserve(points.size());
    for (const auto& p : points)
        points_.emplace_back(std::shared_ptr<Point>(new Point(f, p.first, p.second)));

    cube = std::shared_ptr<Cube>(new Cube(f));

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glEnable(GL_PROGRAM_POINT_SIZE);

    updateCamera();
}

void OpenGLWindow::resizeGL(int w, int h) {
    m_proj = PerspectiveProjection(float(h) / float(w), M_PI / 2.0f, 50.0f, 0.01f);

    program.bind();
    program.setUniformValue(u_proj, m_proj);
}

void OpenGLWindow::paintGL() {
    glClearColor(0, 0, 0, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    program.bind();

    program.setUniformValue(u_grid, false);

    glEnable(GL_PROGRAM_POINT_SIZE);
    program.setUniformValue(u_trans, Identity());
    program.setUniformValue(u_color, QVector4D(1.0f, 0.0f, 0.0f, 1.0f));
    cube->Render();

    program.setUniformValue(u_trans, matrices[cur_trans].second * matrices[cur_trans].first);
    program.setUniformValue(u_color, QVector4D(0.0f, 1.0f, 0.0f, 1.0f));
    for (const auto& p : points_)
        p->Render();

    glDisable(GL_PROGRAM_POINT_SIZE);
    program.setUniformValue(u_color, QVector4D(1.0f, 1.0f, 1.0f, 1.0f));

    for (const auto& p : points_)
        p->RenderNormal();

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    program.setUniformValue(u_grid, true);
    program.setUniformValue(u_trans, grid->Matrix());
    grid->Render();
    program.setUniformValue(u_grid, false);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
}

void OpenGLWindow::mouseMoveEvent(QMouseEvent* event) {

    if (draggedRight && event->buttons().testFlag(Qt::RightButton)) {
        const QPointF diff = lastMousePoint - event->pos();
        if (diff.manhattanLength() < 10) {
            return;
        }
        lastMousePoint = event->pos();
        const float s = std::max(width(), height());
        const float y_angle = 2 * M_PI * diff.x() / s;
        const float x_angle = 2 * M_PI * diff.y() / s;
        camera.RotateX(x_angle);
        camera.RotateY(y_angle);
        updateCamera();
    }
}

void OpenGLWindow::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::RightButton) {
        draggedRight = true;
        lastMousePoint = event->pos();
    }
}

void OpenGLWindow::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::RightButton) {
        draggedRight = false;
        lastMousePoint = QPoint(-1, -1);
    }
}

void OpenGLWindow::keyPressEvent(QKeyEvent* event) {
    switch (event->key()) {
    case Qt::Key::Key_Up: {
        camera.translation.setZ(camera.translation.z() + 0.1f);
        updateCamera();
        break;
    }
    case Qt::Key::Key_Down: {
        camera.translation.setZ(camera.translation.z() - 0.1f);
        updateCamera();
        break;
    }
    case Qt::Key::Key_Right: {
        cur_trans = (cur_trans + 1) % matrices.size();
        update();
        break;
    }
    case Qt::Key::Key_Left: {
        cur_trans = (cur_trans - 1 + matrices.size()) % matrices.size();
        update();
        break;
    }
    }
}

void OpenGLWindow::wheelEvent(QWheelEvent* event) {
    float d = camera.translation.z();
    d += event->pixelDelta().y() / 100.0f;
    if (abs(d) < 0.1f)
        d = -0.1f;
    camera.translation.setZ(d);
    updateCamera();
}
