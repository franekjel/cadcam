#include "ICP.h"

#include <cstdio>

#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

std::vector<std::pair<TopoDS_Vertex, TopoDS_Vertex>> GetVertexPairList(const std::vector<TopoDS_Shape>& points) {
    std::vector<std::pair<TopoDS_Vertex, TopoDS_Vertex>> edges;
    for (TopoDS_Iterator i(TopoDS::Compound(points[0])); i.More(); i.Next()) {
        const TopoDS_Shape& curShape = i.Value();
        for (TopoDS_Iterator it(curShape); it.More(); it.Next()) {
            auto obj = it.Value();
            TopAbs_ShapeEnum type = obj.ShapeType();
            if (type == TopAbs_EDGE) {
                TopoDS_Edge edge = TopoDS::Edge(obj);
                TopoDS_Iterator itt(edge);
                TopoDS_Vertex v1 = TopoDS::Vertex(itt.Value());
                itt.Next();
                TopoDS_Vertex v2 = TopoDS::Vertex(itt.Value());
                edges.push_back({ v1, v2 });
            }
        }
    }
    return edges;
}

std::tuple<Standard_Real, gp_Vec, gp_Pnt> DistAndNormal(const TopoDS_Shape& model, const TopoDS_Vertex& point) {
    BRepExtrema_DistShapeShape tool;
    tool.LoadS1(model);
    tool.LoadS2(point);
    tool.Perform();
    if (!tool.IsDone())
        throw std::runtime_error("BRepExtrema_DistShapeShape error");

    for (int i = 0; i < tool.NbSolution(); i++) {
        if (BRepExtrema_IsInFace == tool.SupportTypeShape1(i)) {
            double u, v;
            tool.ParOnFaceS1(i, u, v);
            TopoDS_Face face = TopoDS::Face(tool.SupportOnShape1(i));
            auto surface = BRep_Tool::Surface(face);
            gp_Pnt p;
            gp_Vec D1U, D1V;
            surface->D1(u, v, p, D1U, D1V);
            gp_Vec N = D1U.Crossed(D1V);
            return { tool.Value(), N, p };
        }
    }

    //return { tool.Value(), gp_Vec(1, 1, 1), gp_Pnt(0, 0, 0) };
    //to się nie powinno stać
}

Eigen::Matrix<double, 6, 1> Gradient(const TopoDS_Shape& model, const std::pair<TopoDS_Vertex, TopoDS_Vertex>& point, const Eigen::Matrix3f& rotation, const Eigen::Matrix3f& translation, const float alpha) {
    Eigen::Matrix<double, 6, 1> re;
    auto p = BRep_Tool::Pnt(point.first);
    Eigen::Vector3d v = Eigen::Vector3d(p.X(), p.Y(), p.Z());
    v = translation * rotation * v;
    BRep_Builder builder;
    TopoDS_Vertex vert;
    builder.MakeVertex(vert, gp_Pnt(v.x(), v.y(), v.z()), 0.01f);
    auto [dist, N, pnt] = DistAndNormal(model, vert);
    p = BRep_Tool::Pnt(point.second);
    Eigen::Vector3d n = Eigen::Vector3d(p.X(), p.Y(), p.Z());
    re(0) = 2 * alpha * dist * N.X();
    re(1) = 2 * alpha * dist * N.Y();
    re(2) = 2 * alpha * dist * N.Z();
    re(3) = 2 * alpha * dist * (N.Z() * (v.y() - pnt.Y()) - N.Y() * (v.z() - pnt.Z())) + (1.0f - alpha) * (n.y() * N.Z() - n.z() * N.Z());
    //TODO: uzupełnić 4 i 5 analogicznie jeśli 3 jest ok.
    return re;
}

std::pair<Eigen::Matrix3Xf, Eigen::Matrix3Xf> ICP(const std::vector<TopoDS_Shape>& model, const std::vector<TopoDS_Shape>& points) {
    const int N = 400;
    const float alpha = 0.8f;

    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f translation = Eigen::Matrix3f::Identity();

    auto edges = GetVertexPairList(points);
    TopoDS_Compound compound;
    TopoDS_Builder builder;
    builder.MakeCompound(compound);
    for (int i = 0; i < model.size(); ++i)
        builder.Add(compound, model[i]);

    /*for (const auto& e : edges) {
        const auto& p1 = BRep_Tool::Pnt(e.first);
        const auto& p2 = BRep_Tool::Pnt(e.second);
        printf("(%lf, %lf, %lf)-(%lf, %lf, %lf)\n", p1.X(), p1.Y(), p1.Z(), p2.X(), p2.Y(), p2.Z());
    }*/

    //for (int i = 0; i < N; i++) {
    //}

    return {};
}
