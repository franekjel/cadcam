#include "torus.h"

void Torus::contructTorus() {

    vertices.clear();
    edgeIndices.clear();
    triangleIndices.clear();

    if (rPoints < 3)
        rPoints = 3;
    if (RPoints < 3)
        RPoints = 3;
    float rStep = 2 * M_PI / rPoints;
    float RStep = 2 * M_PI / RPoints;

    for (int i = 0; i < RPoints; i++) {
        float Angle = i * RStep;
        for (int j = 0; j < rPoints; j++) {
            float angle = j * rStep;
            vertices.push_back(QVector3D(
                (R + r * cos(angle)) * cos(Angle),
                r * sin(angle),
                (R + r * cos(angle)) * sin(Angle)));
        }
    }
    for (unsigned int i = 0; i < RPoints; i++) {
        for (unsigned int j = 0; j < rPoints; j++) {
            edgeIndices.push_back({ i * rPoints + j, i * rPoints + ((j + 1) % rPoints) });
            edgeIndices.push_back({ i * rPoints + j, ((i + 1) * rPoints + j) % (RPoints * rPoints) });
        }
    }

    for (unsigned int i = 0; i < RPoints; i++) {
        for (unsigned int j = 0; j < rPoints; j++) {
            triangleIndices.push_back({ i * rPoints + j, i * rPoints + ((j + 1) % rPoints), ((i + 1) * rPoints + j) % (RPoints * rPoints) });
            triangleIndices.push_back({ i * rPoints + ((j + 1) % rPoints), ((i + 1) * rPoints + j) % (RPoints * rPoints), ((i + 1) * rPoints + (j + 1) % rPoints) % (RPoints * rPoints) });
        }
    }
}

Torus::Torus(float r_, float R_, int rPoints_, int RPoints_, QOpenGLFunctions_4_2_Core* f)
    : Object(f) {
    r = r_;
    R = R_;
    rPoints = rPoints_;
    RPoints = RPoints_;
    contructTorus();

    ModifyOpenglData();
}

void Torus::recreateTorus() {
    contructTorus();
    ModifyOpenglData();
}
