#include "object.h"

#include <QFile>
#include <iostream>
#include <sstream>

QMatrix4x4 Transformation::Matrix() {
    return Translation(translation) * Rotation() * Scale(scaleVector);
}

QMatrix4x4 Transformation::Rotation() {

    return M3to4(rotation.toRotationMatrix());
}

void Object::ModifyOpenglData() {

    f->glBindVertexArray(VAO);

    f->glBindBuffer(GL_ARRAY_BUFFER, VBO);
    f->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size(), vertices.data(), GL_STATIC_DRAW);

    f->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    f->glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indices.size(), indices.data(), GL_STATIC_DRAW);

    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    f->glEnableVertexAttribArray(0);

    f->glBindBuffer(GL_ARRAY_BUFFER, 0);
    f->glBindVertexArray(0);
}

Object::Object(const std::vector<float>& p, const IndicesBuffer& i, QOpenGLFunctions_4_2_Core* f)
    : vertices(p)
    , indices(i)
    , f(f) {

    f->glGenVertexArrays(1, &VAO);
    f->glGenBuffers(1, &VBO);
    f->glGenBuffers(1, &EBO);

    ModifyOpenglData();
}

Object::Object(QOpenGLFunctions_4_2_Core* f)
    : f(f) {
    f->glGenVertexArrays(1, &VAO);
    f->glGenBuffers(1, &VBO);
    f->glGenBuffers(1, &EBO);
}

Object::~Object() {
    f->glDisableVertexAttribArray(0);
    f->glDeleteVertexArrays(1, &VAO);
    f->glDeleteBuffers(1, &VBO);
    f->glDeleteBuffers(1, &EBO);
}

Cube::Cube(QOpenGLFunctions_4_2_Core* f)
    : Cube(
        { -1.0f, 1.0f, -1.0f,
            -1.0f, -1.0f, -1.0f,
            +1.0f, -1.0f, -1.0f,
            +1.0f, -1.0f, -1.0f,
            +1.0f, +1.0f, -1.0f,
            -1.0f, +1.0f, -1.0f,

            -1.0f, -1.0f, +1.0f,
            -1.0f, -1.0f, -1.0f,
            -1.0f, +1.0f, -1.0f,
            -1.0f, +1.0f, -1.0f,
            -1.0f, +1.0f, +1.0f,
            -1.0f, -1.0f, +1.0f,

            +1.0f, -1.0f, -1.0f,
            +1.0f, -1.0f, +1.0f,
            +1.0f, +1.0f, +1.0f,
            +1.0f, +1.0f, +1.0f,
            +1.0f, +1.0f, -1.0f,
            +1.0f, -1.0f, -1.0f,

            -1.0f, -1.0f, +1.0f,
            -1.0f, +1.0f, +1.0f,
            +1.0f, +1.0f, +1.0f,
            +1.0f, +1.0f, +1.0f,
            +1.0f, -1.0f, +1.0f,
            -1.0f, -1.0f, +1.0f,

            -1.0f, +1.0f, -1.0f,
            +1.0f, +1.0f, -1.0f,
            +1.0f, +1.0f, +1.0f,
            +1.0f, +1.0f, +1.0f,
            -1.0f, +1.0f, +1.0f,
            -1.0f, +1.0f, -1.0f,

            -1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f, +1.0f,
            +1.0f, -1.0f, -1.0f,
            +1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f, +1.0f,
            +1.0f, -1.0f, +1.0f },
        { 0, 1, 2, 3, 4, 5,
            6, 7, 8, 9, 10, 11,
            12, 13, 14, 15, 16, 17,
            18, 19, 20, 21, 22, 23,
            24, 25, 26, 27, 28, 29,
            30, 31, 32, 33, 34, 35 },
        f) {
}

void Object::Render() {
    f->glBindVertexArray(VAO);
    f->glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    f->glBindVertexArray(0);
}

void Object::translate(QVector3D v) {
    translation += v;
}

void Object::scale(QVector3D s, QVector3D p) {
    scaleVector += s;
    translation += (translation - p) * s;
}

void Camera::RotateX(float angle) {
    rotationX += angle;
    inversedRotation = Rotation().inverted();
}

void Camera::RotateY(float angle) {
    rotationY += angle;
    inversedRotation = Rotation().inverted();
}

QMatrix4x4 Camera::Matrix() {
    return Translation(-translation) * RotationX(rotationX) * RotationY(rotationY) * Translation(center);
}

QMatrix4x4 Camera::Rotation() {
    return RotationX(rotationX) * RotationY(rotationY);
}

Cube::Cube(const std::vector<float>& p, const IndicesBuffer& i, QOpenGLFunctions_4_2_Core* _f) {
    f = _f;
    indices = i;
    cube_vertices = p;

    f->glGenVertexArrays(1, &VAO);
    f->glGenBuffers(1, &VBO);
    f->glGenBuffers(1, &EBO);
    f->glBindVertexArray(VAO);

    f->glBindBuffer(GL_ARRAY_BUFFER, VBO);
    f->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * cube_vertices.size(), cube_vertices.data(), GL_STATIC_DRAW);

    f->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    f->glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(EdgeIndices) * indices.size(), indices.data(), GL_STATIC_DRAW);

    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    f->glEnableVertexAttribArray(0);

    f->glBindBuffer(GL_ARRAY_BUFFER, 0);
    f->glBindVertexArray(0);
}

Point::Point(QOpenGLFunctions_4_2_Core* f, QVector3D p, QVector3D n)
    : Object(f) {
    vertices = { p.x(), p.y(), p.z(), p.x() + n.x() / 10.0f, p.y() + n.y() / 10.0f, p.z() + n.z() / 10.0f };
    indices = { 0, 1 };
    ModifyOpenglData();
}

void Point::Render() {
    f->glBindVertexArray(VAO);
    f->glDrawElements(GL_POINTS, 1, GL_UNSIGNED_INT, 0);
    f->glBindVertexArray(0);
}

void Point::RenderNormal() {
    f->glBindVertexArray(VAO);
    f->glDrawElements(GL_LINES, indices.size(), GL_UNSIGNED_INT, 0);
    f->glBindVertexArray(0);
}
