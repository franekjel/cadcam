#pragma once
#include <array>
#include <vector>

#include <QMatrix4x4>
#include <QOpenGLFunctions_4_2_Core>
#include <QQuaternion>
#include <QVector3D>

#include "functions.h"

struct TriangleIndices {
    unsigned int indices[3];
};

struct EdgeIndices {
    unsigned int indices[2];
};

struct IndicesBuffer : public std::vector<unsigned int> {
    void add_edge(const EdgeIndices& e) {
        emplace_back(e.indices[0]);
        emplace_back(e.indices[1]);
    }
    void add_triangle(const TriangleIndices& t) {
        emplace_back(t.indices[0]);
        emplace_back(t.indices[1]);
        emplace_back(t.indices[2]);
    }
    void push_back(std::initializer_list<unsigned int> indices) {
        for (const auto& p : indices) {
            emplace_back(p);
        }
    }

    IndicesBuffer(std::initializer_list<unsigned int> init)
        : std::vector<unsigned int>(init) { }
    IndicesBuffer()
        : std::vector<unsigned int>() { }
};

enum DrawMode {
    Edges,
    Triangles,
    Points
};

class Transformation {
public:
    QVector3D translation = QVector3D(0.0f, 0.0f, 0.0f);
    QQuaternion rotation;
    QVector3D scaleVector = QVector3D(1.0f, 1.0f, 1.0f);

    QMatrix4x4 Matrix();
    QMatrix4x4 Rotation();
};

class Object : public Transformation {
public:
    Object(const std::vector<float>& p, const IndicesBuffer& i, QOpenGLFunctions_4_2_Core* f);
    Object(QOpenGLFunctions_4_2_Core* f);
    ~Object();

    static Object* Square(QOpenGLFunctions_4_2_Core* f);

    virtual void Render();

    virtual void translate(QVector3D v);
    virtual void scale(QVector3D s, QVector3D p);

    std::string name = "";
    bool selected = false;

    std::vector<float> vertices;
    IndicesBuffer indices;
    void ModifyOpenglData();

    QOpenGLFunctions_4_2_Core* f;
    unsigned int VAO, VBO, EBO;

protected:
    Object() {};
};

class Cube : public Object {

public:
    Cube(const std::vector<float>& p, const IndicesBuffer& i, QOpenGLFunctions_4_2_Core* f);
    Cube(QOpenGLFunctions_4_2_Core* f);
};

class Point : public Object {

public:
    Point(QOpenGLFunctions_4_2_Core* f, QVector3D p, QVector3D n);
    virtual void Render() override;
    void RenderNormal();
};

class Camera : public Transformation {
public:
    QVector3D center = QVector3D(0, 0, 0);
    float rotationY = 0;
    float rotationX = 0;
    QMatrix4x4 inversedRotation = Identity();
    Camera() {};
    ~Camera() {};
    void RotateX(float angle);
    void RotateY(float angle);
    QMatrix4x4 Matrix();
    QMatrix4x4 Rotation();
};
