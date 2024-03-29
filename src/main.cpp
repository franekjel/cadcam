#include <fstream>
#include <iostream>
#include <vector>

#include "ICP.h"

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>

std::vector<TopoDS_Shape> ReadShape(const char* filename)
{
    STEPControl_Reader reader;
    IFSelect_ReturnStatus stat = reader.ReadFile(filename);
    if (stat != IFSelect_RetDone)
    {
        std::cout << "Error reading model: " << stat << std::endl;
        throw std::runtime_error("wrong model");
    }
    Standard_Integer num = reader.TransferRoots();

    std::vector<TopoDS_Shape> objects(num);
    for (int i = 0; i < num; i++)
    {
        objects[i] = reader.OneShape();
    }
    return objects;
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>> Triangulate(const std::vector<TopoDS_Shape>& parts, double deflection)
{
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector3i> indexes;
    int idx = 0;

    for (auto& part : parts) {
        BRepMesh_IncrementalMesh mesh(part, deflection);
        // iterate over faces:
        for (TopExp_Explorer faceExplorer(part, TopAbs_FACE); faceExplorer.More(); faceExplorer.Next())
        {
            const auto& face = TopoDS::Face(faceExplorer.Current());
            if (face.IsNull())
                continue;

            // triangulate face:
            TopLoc_Location location = part.Location();
            Handle_Poly_Triangulation triangulation = BRep_Tool::Triangulation(face, location);
            if (triangulation.IsNull())
                continue;

            const TColgp_Array1OfPnt& nodes = triangulation->Nodes();
            const Poly_Array1OfTriangle& triangles = triangulation->Triangles();

            const gp_Trsf& transformation = location.Transformation();

            // Iterate over the triangles and their nodes.
            for (int i = triangles.Lower(); i <= triangles.Upper(); i++)
            {
                const Poly_Triangle& triangle = triangles(i);
                const gp_Pnt& p1 = nodes(triangle(1));
                gp_Pnt v1 = p1.Transformed(transformation);
                const gp_Pnt& p2 = nodes(triangle(2));
                gp_Pnt v2 = p2.Transformed(transformation);
                const gp_Pnt& p3 = nodes(triangle(3));
                gp_Pnt v3 = p3.Transformed(transformation);

                positions.emplace_back(v1.X(), v1.Y(), v1.Z());
                positions.emplace_back(v2.X(), v2.Y(), v2.Z());
                positions.emplace_back(v3.X(), v3.Y(), v3.Z());

                indexes.emplace_back(idx, idx + 1, idx + 2);
                idx += 3;
            }
        }
    }

    return std::pair(positions, indexes);
}

int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        std::cout << "MODELS" << std::endl;
        return EXIT_FAILURE;
    }

    auto model = ReadShape(argv[1]);
    auto points = ReadShape(argv[2]);

    auto triangulation = Triangulate(model, 1); //greater deflection -> less triangles, deflection = 10 -> 82 triangles

    std::ofstream file("model.txt", std::ios::trunc);

    file << triangulation.first.size() << std::endl;
    for (const auto& vertex : triangulation.first)
    {
        file << vertex[0] << " " << vertex[1] << " " << vertex[2] << std::endl;
    }
    file << triangulation.second.size() << std::endl;
    for (const auto& indices : triangulation.second)
    {
        file << indices[0] << " " << indices[1] << " " << indices[2] << std::endl;
    }

    ICP(model, points);
}
