#pragma once
#include "object.h"

class Point : public Object {
public:
    Point(QOpenGLFunctions_4_2_Core* f)
        : Object(f) {
        mode = Points;
        vertices = { QVector3D(0, 0, 0), QVector3D(0, 0, 0) };
        edgeIndices = { { 0, 1 } };
        ModifyOpenglData();
    }
};
