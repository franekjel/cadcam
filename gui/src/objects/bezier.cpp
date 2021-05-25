#include "bezier.h"

#include <iostream>

void Bezier::addPoint(std::shared_ptr<Point> point) {
    points.push_back(point);
    edgeIndices.reserve(points.size());
}

void Bezier::Render() {
    f->glBindVertexArray(VAO);
    f->glDrawElements(GL_LINE_STRIP_ADJACENCY, edgeIndices.size(), GL_UNSIGNED_INT, 0);
    f->glBindVertexArray(0);
}

void Bezier::RenderLines() { //must be called with normal shader
    polygonalChain.Render();
    f->glBindVertexArray(polygonalChain.VAO);
    f->glDrawElements(GL_POINTS, polygonalChain.edgeIndices.size(), GL_UNSIGNED_INT, 0);
    f->glBindVertexArray(0);
}

int Bezier::getPointCount() { return points.size(); }

void Bezier::recreateData() {
    vertices.clear();
    edgeIndices.clear();
    for (const auto& point : points)
        vertices.push_back(point->translation);
    while (vertices.size() > 1 && (vertices.size() - 1) % 3 != 0) {
        vertices.push_back(points.back()->translation);
    }
    for (unsigned int i = 0; i < vertices.size(); i++)
        edgeIndices.push_back({ i });
    ModifyOpenglData();

    polygonalChain.vertices.clear();
    for (const auto& point : points)
        polygonalChain.vertices.push_back(point->translation);
    polygonalChain.edgeIndices.clear();
    for (unsigned int i = 1; i < points.size(); i++)
        polygonalChain.edgeIndices.add_edge({ i - 1, i });
    polygonalChain.ModifyOpenglData();
}

void BezierC2::translateBernstainPoint(const int idx, const QVector3D position) {
    int boor_idx = (idx + 2) / 3;
    if (boor_idx < 0 || boor_idx >= points.size())
        return;
    const QVector3D translation = position - vertices[idx];
    if (idx % 3 != 0)
        points[boor_idx]->translation += 1.5f * translation;
    if (boor_idx < points.size() - 1)
        points[boor_idx + 1]->translation += 1.5f * translation;
}

int BezierC2::getPointCount() {
    return (points.size() - 3) * 3 + 1;
}

void BezierC2::recreateData() {
    //modify chain for non-bernstain

    if (!drawBernstain) {
        polygonalChain.vertices.clear();
        for (const auto& point : points)
            polygonalChain.vertices.push_back(point->translation);
        polygonalChain.edgeIndices.clear();
        for (unsigned int i = 1; i < points.size(); i++)
            polygonalChain.edgeIndices.add_edge({ i - 1, i });
        polygonalChain.ModifyOpenglData();
    }

    vertices.clear();
    edgeIndices.clear();
    if (points.size() < 4)
        return;
    for (unsigned int i = 0; i < points.size() - 3; i++) {
        const QVector3D p1 = points[i]->translation + 2.0f * ((points[i + 1]->translation - points[i]->translation) / 3.0f);
        const QVector3D p2 = points[i + 1]->translation + (points[i + 2]->translation - points[i + 1]->translation) / 3.0f;
        const QVector3D p3 = points[i + 1]->translation + 2.0f * ((points[i + 2]->translation - points[i + 1]->translation) / 3.0f);
        vertices.emplace_back((p1 + p2) / 2.0f);
        edgeIndices.push_back({ (unsigned int)vertices.size() - 1 });
        vertices.emplace_back(p2);
        edgeIndices.push_back({ (unsigned int)vertices.size() - 1 });
        vertices.emplace_back(p3);
        edgeIndices.push_back({ (unsigned int)vertices.size() - 1 });
    }
    const int ps = points.size();
    const QVector3D p3 = points[ps - 3]->translation + 2.0f * ((points[ps - 2]->translation - points[ps - 3]->translation) / 3.0f);
    const QVector3D p4 = points[ps - 2]->translation + (points[ps - 1]->translation - points[ps - 2]->translation) / 3.0f;
    vertices.emplace_back((p3 + p4) / 2.0f);
    edgeIndices.push_back({ (unsigned int)vertices.size() - 1 });
    ModifyOpenglData();

    if (drawBernstain) { //modify chain for bernstain
        polygonalChain.vertices.clear();
        for (const auto& vertice : vertices)
            polygonalChain.vertices.push_back(vertice);
        polygonalChain.edgeIndices.clear();
        for (unsigned int i = 1; i < vertices.size(); i++)
            polygonalChain.edgeIndices.add_edge({ i - 1, i });
        polygonalChain.ModifyOpenglData();
    }
}

void InterpolationBezierC2::recreateData() {
    if (!drawBernstain) {
        polygonalChain.vertices.clear();
        for (const auto& point : points)
            polygonalChain.vertices.push_back(point->translation);
        polygonalChain.edgeIndices.clear();
        for (unsigned int i = 1; i < points.size(); i++)
            polygonalChain.edgeIndices.add_edge({ i - 1, i });
        polygonalChain.ModifyOpenglData();
    }

    vertices.clear();
    edgeIndices.clear();
    if (points.size() < 2)
        return;
    else {
        std::vector<QVector3D> control_points;
        control_points.reserve(points.size());
        control_points.push_back(points[0]->translation);
        for (int i = 0; i < points.size() - 1; i++) {
            float D = points[i]->translation.distanceToPoint(points[i + 1]->translation);
            if (D > 0.01f)
                control_points.push_back(points[i + 1]->translation);
        }

        const int N = control_points.size() - 1;
        std::vector<float> D(N); //dists, bold d_i from lecture
        for (int i = 0; i < N; i++)
            D[i] = control_points[i].distanceToPoint(control_points[i + 1]);

        std::vector<float> t(N + 1);
        t[0] = 0;
        for (int i = 0; i <= N - 1; i++)
            t[i + 1] = t[i] + D[i];

        std::vector<float> alpha(N);
        for (int i = 1; i <= N - 1; i++)
            alpha[i] = D[i - 1] / (D[i - 1] + D[i]);

        std::vector<float> beta(N);
        for (int i = 1; i <= N - 1; i++)
            beta[i] = D[i] / (D[i - 1] + D[i]);

        std::vector<QVector3D> R(N);
        for (int i = 1; i <= N - 1; i++) {
            // clang-format off
            R[i] = 3.0f * (
                    (
                             (control_points[i + 1] - control_points[i]) / D[i]
                           - (control_points[i] - control_points[i - 1]) / D[i - 1]
                    ) / (D[i - 1] + D[i]));
            // clang-format on
        }
        std::vector<QVector3D> c(N + 1);
        c[0] = QVector3D(0.0f, 0.0f, 0.0f);
        c[N] = QVector3D(0.0f, 0.0f, 0.0f);
        std::vector<float> diag(N + 1, 2.0f);
        for (int i = 2; i <= N - 1; i++) { //clear below diagonal
            const float factor = (alpha[i] / diag[i - 1]);
            diag[i] -= factor * beta[i - 1];
            R[i] -= factor * R[i - 1];
        }
        for (int i = N - 1; i >= 2; i--) { //clear abowe diagonal
            R[i] /= diag[i];
            diag[i] = 1.0f;
            c[i] = R[i];
            R[i - 1] -= beta[i - 1] * R[i];
        }
        R[1] /= diag[1];
        c[1] = R[1];

        std::vector<QVector3D> a(N + 1);
        for (int i = 0; i <= N; i++)
            a[i] = control_points[i];

        std::vector<QVector3D> d(N + 1);
        for (int i = 1; i <= N + 1; i++)
            d[i - 1] = (c[i] - c[i - 1]) / (3.0f * D[i - 1]);

        std::vector<QVector3D> b(N + 1);
        for (int i = 1; i <= N; i++)
            b[i - 1] = (a[i] - a[i - 1]) / D[i - 1] - (c[i - 1] + d[i - 1] * D[i - 1]) * D[i - 1];

        for (int i = 0; i <= N; i++) { //scaling to [0,1]
            b[i] *= D[i];
            c[i] *= D[i] * D[i];
            d[i] *= D[i] * D[i] * D[i];
        }
        QVector4D bezier_points[3];
        for (int i = 0; i <= N - 1; i++) {
            for (int j = 0; j < 3; j++)
                bezier_points[j] = ToBezierBase() * QVector4D(a[i][j], b[i][j], c[i][j], d[i][j]);
            for (int j = 0; j < 3; j++)
                vertices.push_back(QVector3D(bezier_points[0][j], bezier_points[1][j], bezier_points[2][j]));
        }
        vertices.push_back(QVector3D(bezier_points[0][3], bezier_points[1][3], bezier_points[2][3]));

        for (unsigned int i = 0; i < vertices.size(); i++)
            edgeIndices.push_back({ i });
    }
    ModifyOpenglData();

    if (drawBernstain) { //modify chain for bernstain
        polygonalChain.vertices.clear();
        for (const auto& vertice : vertices)
            polygonalChain.vertices.push_back(vertice);
        polygonalChain.edgeIndices.clear();
        for (unsigned int i = 1; i < vertices.size(); i++)
            polygonalChain.edgeIndices.add_edge({ i - 1, i });
        polygonalChain.ModifyOpenglData();
    }
}

int InterpolationBezierC2::getPointCount() {
    return vertices.size();
}

QVector3D InterpolationBezierC2::half(int i) {
    recreateData();
    return DeCasteljauCubic({ vertices[0 + i * 3], vertices[1 + i * 3], vertices[2 + i * 3], vertices[3 + i * 3] }, 0.5f);
}

void InterpolationBezierC2::AddAfter(int i, std::shared_ptr<Point> p) {
    points.insert(points.begin() + 1 + i, p);
    recreateData();
}
