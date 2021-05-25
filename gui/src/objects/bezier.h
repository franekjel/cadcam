#pragma once
#include "object.h"
#include "point.h"

enum BezierType {
    C0,
    C2,
    InterpolationC2
};

class Bezier : public Object {
public:
    std::vector<std::shared_ptr<Point>> points;

    bool drawLines = true;
    BezierType type;

    Object polygonalChain;

    Bezier(QOpenGLFunctions_4_2_Core* f)
        : Object(f)
        , polygonalChain(f) { }
    void addPoint(std::shared_ptr<Point> point);
    virtual void Render() override;
    void RenderLines();
    virtual void translate(QVector3D v) override final {};
    virtual void scale(QVector3D s, QVector3D p) override final {};
    virtual int getPointCount();
    virtual void recreateData();
};

class BezierC0 : public Bezier {
public:
    BezierC0(QOpenGLFunctions_4_2_Core* f)
        : Bezier(f) { type = C0; }
};

class BezierC2 : public Bezier {
public:
    void translateBernstainPoint(const int idx, const QVector3D position);

    bool drawBernstain = false;

    virtual int getPointCount() override;
    virtual void recreateData() override;

    BezierC2(QOpenGLFunctions_4_2_Core* f)
        : Bezier(f) { type = C2; }
};

class InterpolationBezierC2 : public Bezier {
public:
    InterpolationBezierC2(QOpenGLFunctions_4_2_Core* f)
        : Bezier(f) { type = InterpolationC2; }
    bool drawBernstain = false;
    virtual void recreateData() override;
    virtual int getPointCount() override;
    QVector3D half(int i);
    void AddAfter(int i, std::shared_ptr<Point> p);
};
