#include "ICP.h"

#include <cstdio>
#include <fstream>

#include <Eigen/Geometry>

#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

std::vector<std::pair<TopoDS_Vertex, TopoDS_Vertex>> GetVertexPairList(const std::vector<TopoDS_Shape>& points) {
    std::vector<std::pair<TopoDS_Vertex, TopoDS_Vertex>> edges;
    for (const auto& point : points) {
        TopoDS_Vertex vv;
        int i = 0;
        for (TopExp_Explorer vertexExplorer(point, TopAbs_VERTEX); vertexExplorer.More(); vertexExplorer.Next(), i++) {
            const auto& vertex = TopoDS::Vertex(vertexExplorer.Current());
            if (vertex.IsNull())
                continue;
            if (i % 2 == 0)
                vv = vertex;
            else
                edges.emplace_back(vv, vertex);
        }
    }
    return edges;
}

std::tuple<Standard_Real, gp_Vec, gp_Pnt> DistAndNormal(const TopoDS_Shape& model, const TopoDS_Vertex& point, const gp_Vec normal) {
    BRepExtrema_DistShapeShape tool;
    tool.LoadS1(model);
    tool.LoadS2(point);
    tool.Perform();

    if (!tool.IsDone())
        throw std::runtime_error("BRepExtrema_DistShapeShape error");

    const int n = tool.NbSolution();

    for (int i = 1; i <= n; i++) {
        if (BRepExtrema_IsInFace == tool.SupportTypeShape1(i)) {
            double u, v;
            tool.ParOnFaceS1(i, u, v);
            TopoDS_Face face = TopoDS::Face(tool.SupportOnShape1(i));
            opencascade::handle<Geom_Surface> surface = BRep_Tool::Surface(face);
            gp_Pnt p;
            gp_Vec D1U, D1V;
            surface->D1(u, v, p, D1U, D1V);
            gp_Vec N = (face.Orientation() == TopAbs_REVERSED) ? D1V.Crossed(D1U) : D1U.Crossed(D1V);
            return { tool.Value(), N, p };
        } else if (BRepExtrema_IsOnEdge == tool.SupportTypeShape1(i)) {
            double t;
            tool.ParOnEdgeS1(i, t);
            TopoDS_Edge edge = TopoDS::Edge(tool.SupportOnShape1(i));
            double first, last;
            opencascade::handle<Geom_Curve> curve = BRep_Tool::Curve(edge, first, last);
            gp_Pnt p;
            curve->D0(t, p);
            return { tool.Value(), normal, p };
        } else if (BRepExtrema_IsVertex == tool.SupportTypeShape1(i)) {
            TopoDS_Vertex vertex = TopoDS::Vertex(tool.SupportOnShape1(i));
            gp_Pnt p = BRep_Tool::Pnt(vertex);
            return { tool.Value(), normal, p };
        }
    }

    //return { tool.Value(), gp_Vec(1, 1, 1), gp_Pnt(0, 0, 0) };
    //to się nie powinno stać
}

Eigen::Matrix<double, 6, 1> Gradient(const TopoDS_Shape& model, const std::pair<TopoDS_Vertex, TopoDS_Vertex>& point, const Eigen::Matrix4d& rotation, const Eigen::Matrix4d& translation, const float alpha) {

    /*
    v,n - punkt i normalna po transformacji jako eigenowy wektor
    VV,NN - punkt i normalna po transformacji w formacie cascadowym jako wejście do funkcji dystansu
    V,N - wyjście funkcji dystansu w formacie cascadowym
    */

    Eigen::Matrix<double, 6, 1> re;
    gp_Pnt p = BRep_Tool::Pnt(point.first);
    Eigen::Vector4d v = Eigen::Vector4d(p.X(), p.Y(), p.Z(), 1.0f);
    v = translation * rotation * v;
    v /= v.w();

    gp_Pnt p2 = BRep_Tool::Pnt(point.second);
    Eigen::Vector4d n = Eigen::Vector4d(p2.X(), p2.Y(), p2.Z(), 0.0f);
    n = rotation * n;

    //Eigen::Vector3d n = Eigen::Vector3d(v2.x() - v.x(), v2.y() - v.y(), v2.z() - v.z());
    //n.normalize();

    BRep_Builder builder;
    TopoDS_Vertex VV;
    builder.MakeVertex(VV, gp_Pnt(v.x(), v.y(), v.z()), 0.01f);
    gp_Vec NN = gp_Vec(n.x(), n.y(), n.z());
    NN.Normalize();
    auto [dist, N, V] = DistAndNormal(model, VV, NN);
    re(0) = 2 * alpha * dist * N.X();
    re(1) = 2 * alpha * dist * N.Y();
    re(2) = 2 * alpha * dist * N.Z();
    re(3) = 2 * alpha * dist * (N.Z() * (v.y() - V.Y()) - N.Y() * (v.z() - V.Z())) + (1.0f - alpha) * (n.y() * N.Z() - n.z() * N.Y());
    re(4) = 2 * alpha * dist * (N.X() * (v.z() - V.Z()) - N.Z() * (v.x() - V.X())) + (1.0f - alpha) * (n.z() * N.X() - n.x() * N.Z());
    re(5) = 2 * alpha * dist * (N.Y() * (v.x() - V.X()) - N.X() * (v.y() - V.Y())) + (1.0f - alpha) * (n.x() * N.Y() - n.y() * N.X());
    return re;
}

std::pair<Eigen::Matrix3Xf, Eigen::Matrix3Xf> ICP(const std::vector<TopoDS_Shape>& model, const std::vector<TopoDS_Shape>& points) {
    const int N = 400;
    const float alpha = 0.8f;

    Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();

    std::vector<std::pair<TopoDS_Vertex, TopoDS_Vertex>> edges = GetVertexPairList(points);
    TopoDS_Compound compound;
    TopoDS_Builder builder;
    builder.MakeCompound(compound);
    for (int i = 0; i < model.size(); ++i)
        builder.Add(compound, model[i]);

    std::ofstream file("matrices.txt", std::ios::trunc);
    for (int i = 0; i < 100; i++) {

        Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero(6, 1);
        for (int i = 0; i < edges.size(); i++) {
            g += Gradient(compound, edges[i], rotation, translation, 1.0f) / points.size();
        }
        g /= -double(edges.size());

        Eigen::Transform<double, 3, Eigen::Affine> transformation;
        transformation = Eigen::Translation<double, 3>(g[0], g[1], g[2]);
        translation *= transformation.matrix();
        auto v = translation.col(3);
        file << v.x() << " " << v.y() << " " << v.z();

        double s = abs(g[0]) + abs(g[1]) + abs(g[2]);
        s = abs(g[3]) + abs(g[4]) + abs(g[5]);
        if (s > 0.001f) {
            g[3] /= s;
            g[4] /= s;
            g[5] /= s;
        }

        file << " 1 0 0 0" << std::endl;

        /*
        Trzeba zapisywać do pliku w formacie
        Tx Ty Tz Rx Ry Rz alfa
        Gdzie T to translacja
        R - oś obrotu
        alfa - kąt
        file << ...
        */

        transformation = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

        printf("%lf %lf %lf %lf %lf %lf\n", g[0], g[1], g[2], g[3], g[4], g[5]);
    }

    return {};
}
