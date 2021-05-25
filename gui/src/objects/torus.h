#pragma once
#include "object.h"

class Torus : public Object {
private:
    void contructTorus();

public:
    float r;
    float R;
    int rPoints;
    int RPoints;

    Torus(float r_, float R_, int rPoints_, int RPoints_, QOpenGLFunctions_4_2_Core* f);
    void recreateTorus();
};
