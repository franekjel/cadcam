#include "functions.h"

#include <cmath>

QMatrix4x4 M3to4(const QMatrix3x3& m) {
    return QMatrix4x4(m(0, 0), m(0, 1), m(0, 2), 0, m(1, 0), m(1, 1), m(1, 2), 0, m(2, 0), m(2, 1), m(2, 2), 0, 0, 0, 0, 1);
}

QMatrix3x3 M4to3(const QMatrix4x4& m) {
    const float f[] { m(0, 0), m(0, 1), m(0, 2), m(1, 0), m(1, 1), m(1, 2), m(2, 0), m(2, 1), m(2, 2) };
    return QMatrix3x3(f);
}

QMatrix4x4 Identity() {
    // clang-format off
    QMatrix4x4 re(
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
    );
    // clang-format on
    return re;
}

QMatrix4x4 RotationX(float angle) {
    // clang-format off
    QMatrix4x4 re(
    1, 0, 0, 0,
    0, cos(angle),-sin(angle),0,
    0, sin(angle), cos(angle),0,
    0, 0, 0, 1
    );
    // clang-format on
    return re;
}

QMatrix4x4 RotationY(float angle) {
    // clang-format off
    QMatrix4x4 re(
    cos(angle), 0, sin(angle), 0,
    0, 1, 0, 0,
    -sin(angle), 0, cos(angle),0,
    0, 0, 0, 1
    );
    // clang-format on
    return re;
}

QMatrix4x4 RotationZ(float angle) {
    // clang-format off
    QMatrix4x4 re(
    cos(angle), 0, sin(angle), 0,
    0, 1, 0, 0,
    -sin(angle), 0, cos(angle),0,
    0, 0, 0, 1
    );
    // clang-format on
    return re;
}

QMatrix4x4 Scale(float x, float y, float z) {
    // clang-format off
    QMatrix4x4 re(
    x, 0, 0, 0,
    0, y, 0, 0,
    0, 0, z, 0,
    0, 0, 0, 1
    );
    // clang-format on
    return re;
}

QMatrix4x4 Scale(QVector3D s) {
    return Scale(s.x(), s.y(), s.z());
}

QMatrix4x4 Translation(float x, float y, float z) {
    // clang-format off
    QMatrix4x4 re(
    1, 0, 0, x,
    0, 1, 0, y,
    0, 0, 1, z,
    0, 0, 0, 1
    );
    // clang-format on
    return re;
}

QMatrix4x4 Translation(QVector3D t) {
    return Translation(t.x(), t.y(), t.z());
}

QMatrix4x4 OrthographicProjection(float left, float right, float bottom, float top) {
    // clang-format off
    QMatrix4x4 re(
    2.0f/(right-left), 0, 0, -(right+left)/(right-left),
    0, 2.0f/(top-bottom), 0, -(top+bottom)/(top-bottom),
    0, 0, 0, 0,
    0, 0, 0, 1
    );
    // clang-format on
    return re;
}

QMatrix4x4 PerspectiveProjection(float aspect, float fov, float far, float near) {
    // clang-format off
    QMatrix4x4 re(
    aspect/tan(fov/2.0f), 0, 0, 0,
    0, 1.0f/tan(fov/2.0f), 0, 0,
    0, 0, (far+near)/(far-near), (-2*far*near)/(far-near),
    0, 0, 1, 0
    );
    // clang-format on
    return re;
}

QMatrix4x4 PerspectiveProjection(float far, float near, float left, float right, float bottom, float top) {
    // clang-format off
    QMatrix4x4 re(
    2.0f*near/(right-left), 0, (right+left)/(right-left), 0,
    0, 2.0f*near/(top-bottom), (top+bottom)/(top-bottom), 0,
    0, 0, (far+near)/(far-near), (-2*far*near)/(far-near),
    0, 0, 1, 0
    );
    // clang-format on
    return re;
}

//dist from line p+vt to point q
float DistLineToPoint(QVector3D p, QVector3D v, QVector3D q) {
    return std::abs(QVector3D::crossProduct(q - p, v).length() / v.length());
}

QVector3D DeCasteljauCubic(const std::array<QVector3D, 4>& points, const float t) {
    const float t1 = (1.0f - t);
    QVector3D np[5];
    np[0] = points[0] * t + points[1] * t1;
    np[1] = points[1] * t + points[2] * t1;
    np[2] = points[2] * t + points[3] * t1;

    np[3] = np[0] * t + np[1] * t1;
    np[4] = np[1] * t + np[2] * t1;
    return np[3] * t + np[4] * t1;
}

QVector3D DeCasteljauSquare(const std::array<QVector3D, 3>& points, const float t) {
    const float t1 = (1.0f - t);
    const QVector3D p1 = points[0] * t + points[1] * t1;
    const QVector3D p2 = points[1] * t + points[2] * t1;
    return p1 * t + p2 * t1;
}

QMatrix4x4 ToBezierBase() {
    // clang-format off
    QMatrix4x4 re(
    1, 0, 0, 0,
    1, 1.0f/3.0f, 0, 0,
    1, 2.0f/3.0f, 1.0f/3.0f, 0,
    1, 1, 1, 1
    );
    // clang-format on
    return re;
}
