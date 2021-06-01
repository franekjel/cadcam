#include <iostream>
#include <vector>

#include "ICP.h"

#include <STEPCAFControl_Reader.hxx>
#include <Poly_Triangulation.hxx>
#include <TopoDS_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>

std::vector<TopoDS_Shape> ReadShape(const char* filename) {
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
    return objects;
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>> Triangulate(const std::vector<TopoDS_Shape>& parts, double deflection) {
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector3i> indexes;
    int idx = 0;

    // make one compound object:
    TopoDS_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    for (auto & part : parts) builder.Add(compound, part);
    BRepMesh_IncrementalMesh mesh(compound, deflection);

    // iterate over faces:
    for (TopExp_Explorer faceExplorer(compound, TopAbs_FACE); faceExplorer.More(); faceExplorer.Next()) {
        const auto& face = TopoDS::Face(faceExplorer.Current());
        if (face.IsNull()) continue;

        // triangulate face:
        TopLoc_Location location;
        Handle_Poly_Triangulation triangulation = BRep_Tool::Triangulation (face, location);
        if(triangulation.IsNull()) continue;

        const TColgp_Array1OfPnt& nodes = triangulation->Nodes();
        const Poly_Array1OfTriangle& triangles = triangulation->Triangles();
        // Iterate over the triangles and their nodes.
        for (int i = triangles.Lower(); i <= triangles.Upper(); i++) {
            const Poly_Triangle& triangle = triangles (i);
            const gp_Pnt& p1 = nodes (triangle (1));
            const gp_Pnt& p2 = nodes (triangle (2));
            const gp_Pnt& p3 = nodes (triangle (3));

            positions.emplace_back(p1.X(), p1.Y(), p1.Z());
            positions.emplace_back(p2.X(), p2.Y(), p2.Z());
            positions.emplace_back(p3.X(), p3.Y(), p3.Z());

            indexes.emplace_back(idx, idx + 1, idx + 2);
            idx += 3;
        }
    }

    return std::pair(positions, indexes);
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cout << "MODELS" << std::endl;
        return EXIT_FAILURE;
    }

    auto model = ReadShape(argv[1]);
    auto points = ReadShape(argv[2]);

    auto triangulation = Triangulate(model, 1); //greater deflection -> less triangles, deflection = 10 -> 82 triangles

    std::cout << triangulation.second.size() << std::endl;

    for (const auto &vertex : triangulation.first) {
        std::cout << vertex[0] << ", " << vertex[1] << ", " << vertex[2] << std::endl;
    }

    ICP(model, points);
}
