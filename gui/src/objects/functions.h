#pragma once
#include <cmath>

#include <QMatrix4x4>
#include <QVector3D>

/* All angles in radians
 */

QMatrix4x4 Identity();

QMatrix4x4 RotationX(float angle);
QMatrix4x4 RotationY(float angle);
QMatrix4x4 RotationZ(float angle);

QMatrix4x4 Scale(float x, float y, float z);
QMatrix4x4 Scale(QVector3D s);

QMatrix4x4 Translation(float x, float y, float z);
QMatrix4x4 Translation(QVector3D t);

QMatrix4x4 OrthographicProjection(float left, float right, float bottom, float top);
QMatrix4x4 PerspectiveProjection(float aspect, float fov, float far, float near);
QMatrix4x4 PerspectiveProjection(float far, float near, float left, float right, float bottom, float top);

//dist from line p+vt to point q
float DistLineToPoint(QVector3D p, QVector3D v, QVector3D q);

QMatrix4x4 M3to4(const QMatrix3x3& m);

QMatrix3x3 M4to3(const QMatrix4x4& m);

QVector3D DeCasteljauCubic(const std::array<QVector3D, 4>& points, const float t);

QVector3D DeCasteljauSquare(const std::array<QVector3D, 3>& points, const float t);

QMatrix4x4 ToBezierBase();
