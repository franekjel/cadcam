#include <fstream>
#include <iostream>
#include <string>

#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <QApplication>
#include <STEPCAFControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include "openglwindow.h"

std::vector<std::pair<QVector3D, QVector3D>> ReadPoints(const char* filename) {
    STEPControl_Reader reader;
    IFSelect_ReturnStatus stat = reader.ReadFile(filename);
    if (stat != IFSelect_RetDone) {
        std::cout << "Error reading model: " << stat << std::endl;
        throw std::runtime_error("wrong model");
    }
    Standard_Integer num = reader.TransferRoots();

    std::vector<TopoDS_Shape> objects(num);
    for (int i = 0; i < num; i++) {
        objects[i] = reader.OneShape();
    }

    std::vector<std::pair<QVector3D, QVector3D>> points;
    for (const auto& point : objects) {
        TopoDS_Vertex vv;
        int i = 0;
        for (TopExp_Explorer vertexExplorer(point, TopAbs_VERTEX); vertexExplorer.More(); vertexExplorer.Next(), i++) {
            const auto& vertex = TopoDS::Vertex(vertexExplorer.Current());
            if (vertex.IsNull())
                continue;
            if (i % 2 == 0)
                vv = vertex;
            else {
                gp_Pnt p = BRep_Tool::Pnt(vv);
                gp_Pnt n = BRep_Tool::Pnt(vertex);
                points.emplace_back(QVector3D(p.X(), p.Y(), p.Z()), QVector3D(n.X(), n.Y(), n.Z()));
            }
        }
    }
    return points;
}

std::vector<std::pair<QMatrix4x4, QMatrix4x4>> ReadMatrices(const char* filename) {
    std::ifstream file;
    file.open(filename);
    if (!file.is_open()) {
        std::cout << "Cannot open file " << filename << std::endl;
        throw std::runtime_error("cannot read points");
    }
    std::string line;
    std::vector<std::pair<QMatrix4x4, QMatrix4x4>> matrices;
    while (std::getline(file, line)) {
        if (line.size() < 3)
            continue;
        std::istringstream ss(line);
        QMatrix4x4 tra, rot;
        float x, y, z, angle;
        ss >> x;
        ss >> y;
        ss >> z;
        tra.translate(x, y, z);

        ss >> x;
        ss >> y;
        ss >> z;
        ss >> angle;
        rot.rotate(x, y, z, angle);
        matrices.emplace_back(tra, rot);
    }
    return matrices;
}

int main(int argc, char* argv[]) {

    if (argc < 3) {
        std::cout << "NEED FILES" << std::endl;
        return EXIT_FAILURE;
    }

    std::vector<std::pair<QVector3D, QVector3D>> points = ReadPoints(argv[1]);
    std::vector<std::pair<QMatrix4x4, QMatrix4x4>> matrices = ReadMatrices(argv[2]);

    std::cout << "Found " << points.size() << " points" << std::endl;

    QApplication a(argc, argv);

    QSurfaceFormat f;
    f.setRenderableType(QSurfaceFormat::OpenGL);
    f.setVersion(4, 2);
    f.setProfile(QSurfaceFormat::CoreProfile);
    f.setColorSpace(QSurfaceFormat::sRGBColorSpace);
    f.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    f.setDepthBufferSize(24);
    f.setStencilBufferSize(8);
    QSurfaceFormat::setDefaultFormat(f);

    OpenGLWindow window(points, matrices);
    window.show();
    return a.exec();
}
