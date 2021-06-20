#include "src/cxxopts.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>

#include <TopoDS_Edge.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <STEPControl_StepModelType.hxx>
#include <STEPControl_Writer.hxx>

double Area(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) {
    Eigen::Vector3d v12 = p1 - p2;
    Eigen::Vector3d v13 = p1 - p3;
    return 0.5 * v12.cross(v13).norm();
}

inline double uniform_deviate (int seed)
{
    double ran = seed * (1.0 / (RAND_MAX + 1.0));
    return ran;
}

inline void randomPointTriangle (double a1, double a2, double a3, double b1, double b2, double b3, double c1, double c2, double c3, Eigen::Vector3d& p)
{
    double r1 = static_cast<double> (uniform_deviate (rand ()));
    double r2 = static_cast<double> (uniform_deviate (rand ()));
    double r1sqr = std::sqrt (r1);
    double OneMinR1Sqr = (1 - r1sqr);
    double OneMinR2 = (1 - r2);
    a1 *= OneMinR1Sqr;
    a2 *= OneMinR1Sqr;
    a3 *= OneMinR1Sqr;
    b1 *= OneMinR2;
    b2 *= OneMinR2;
    b3 *= OneMinR2;
    c1 = r1sqr * (r2 * c1 + b1) + a1;
    c2 = r1sqr * (r2 * c2 + b2) + a2;
    c3 = r1sqr * (r2 * c3 + b3) + a3;
    p[0] = c1;
    p[1] = c2;
    p[2] = c3;
}

inline void randPSurface (const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3i>& indices, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector3d& p, Eigen::Vector3d& n)
{
    auto r = static_cast<double> (uniform_deviate (rand ()) * totalArea);

    auto low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
    long idx = low - cumulativeAreas->begin();

    Eigen::Vector3d A = vertices[indices[idx][0]];
    Eigen::Vector3d B = vertices[indices[idx][1]];
    Eigen::Vector3d C = vertices[indices[idx][2]];

    // OBJ: Vertices are stored in a counter-clockwise order by default
    Eigen::Vector3d v1 = A - C;
    Eigen::Vector3d v2 = B - C;
    n = v1.cross(v2);
    n.normalize();

    randomPointTriangle (A[0], A[1], A[2],B[0], B[1], B[2],C[0], C[1], C[2], p);
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> UniformSampling (const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3i>& indices, size_t n_samples) {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> cloud;

    double totalArea = 0;
    std::vector<double>cumulativeAreas (indices.size(), 0);
    size_t i = 0;
    for (auto& t : indices) {
        Eigen::Vector3d v1 = vertices[t[0]];
        Eigen::Vector3d v2 = vertices[t[1]];
        Eigen::Vector3d v3 = vertices[t[2]];
        totalArea += Area(v1, v2, v3);
        cumulativeAreas[i++] = totalArea;
    }

    for (i = 0; i < n_samples; i++)
    {
        Eigen::Vector3d p;
        Eigen::Vector3d n;
        randPSurface (vertices, indices, &cumulativeAreas, totalArea, p, n);
        cloud.emplace_back(p, n);
    }

    return cloud;
}

std::vector<TopoDS_Shape> ReadShape(const char* filename) {
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

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>> Triangulate(const std::vector<TopoDS_Shape>& parts, double deflection) {
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

                if(face.Orientation() == TopAbs_REVERSED) {
                    indexes.emplace_back(idx + 2, idx + 1, idx + 0);
                } else {
                    indexes.emplace_back(idx + 0, idx + 1, idx + 2);
                }

                idx += 3;
            }
        }
    }

    return std::pair(positions, indexes);
}

Eigen::Matrix4d buildTransformation(double tx, double ty, double tz, double rx, double ry, double rz) {
    Eigen::Transform<double, 3, Eigen::Affine> transformation;
    transformation = Eigen::Translation<double, 3>(Eigen::Vector3d(tx, ty, tz));
    rx *= M_PI / 180.0;
    ry *= M_PI / 180.0;
    rz *= M_PI / 180.0;
    transformation.rotate(Eigen::AngleAxis<double>(rx, Eigen::Vector3d::UnitX()));
    transformation.rotate(Eigen::AngleAxis<double>(ry, Eigen::Vector3d::UnitY()));
    transformation.rotate(Eigen::AngleAxis<double>(rz, Eigen::Vector3d::UnitZ()));
    return transformation.matrix();
}

void writePointCloud(const std::string& output, const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& cloud, const Eigen::Matrix4d& transformation) {
    TopoDS_Compound source;
    TopoDS_Builder builder;
    builder.MakeCompound(source);

    for (auto & point : cloud) {
        Eigen::Vector4d vertex = {point.first[0], point.first[1], point.first[2], 1};
        Eigen::Vector4d normal = {point.second[0], point.second[1], point.second[2], 0};
        vertex = transformation * vertex;
        normal = transformation * normal;
        gp_Pnt P(vertex[0], vertex[1], vertex[2]);
        gp_Pnt N(normal[0], normal[1], normal[2]);
        TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(P, N);
        builder.Add(source, edge);
    }

    STEPControl_Writer writer;
    writer.Transfer(source, STEPControl_GeometricCurveSet);
    writer.Write(output.c_str());
}

int main(int argc, char* argv[]) {
    cxxopts::Options options(argv[0], " - command line options");
    options.add_options()
            ("n", "Number of points to generate", cxxopts::value<int>()->default_value("1000"), "N")
            ("d,deflection", "Deflection used during triangulation", cxxopts::value<double>()->default_value("1.0"), "D")
            ("i,input", "Input file(STEP model)", cxxopts::value<std::string>(), "input.step")
            ("o,output", "Output file(STEP model)", cxxopts::value<std::string>()->default_value("output.step"), "output.step")
            ("x,translationX", "Translation in X direction", cxxopts::value<double>()->default_value("0.0"), "Tx")
            ("y,translationY", "Translation in X direction", cxxopts::value<double>()->default_value("0.0"), "Ty")
            ("z,translationZ", "Translation in X direction", cxxopts::value<double>()->default_value("0.0"), "Tz")
            ("X,rotationX", "Rotation around X axis in degrees(euler angles ZYX)", cxxopts::value<double>()->default_value("0.0"), "Rx")
            ("Y,rotationY", "Rotation around Y axis in degrees(euler angles ZYX)", cxxopts::value<double>()->default_value("0.0"), "Ry")
            ("Z,rotationZ", "Rotation around Z axis in degrees(euler angles ZYX)", cxxopts::value<double>()->default_value("0.0"), "Rz")
            ("h,help", "Print help");

    try {
        auto result = options.parse(argc, argv);

        if (result.count("help") || !result.count("i")) {
            std::cout << options.help() << std::endl;
            exit(0);
        }

        int n = result["n"].as<int>();
        double d = result["d"].as<double>();
        std::string input = result["i"].as<std::string>();
        std::string output = result["o"].as<std::string>();
        double tx = result["x"].as<double>();
        double ty = result["y"].as<double>();
        double tz = result["z"].as<double>();
        double rx = result["X"].as<double>();
        double ry = result["Y"].as<double>();
        double rz = result["Z"].as<double>();

        auto step = ReadShape(input.c_str());
        auto [vertices, indices] = Triangulate(step, d);

        auto cloud = UniformSampling(vertices, indices, n);
        auto transformation = buildTransformation(tx, ty, tz, rx, ry, rz);

        writePointCloud(output, cloud, transformation);

    } catch (const cxxopts::OptionException& e) {
        std::cerr << "Error parsing options: " << e.what() << std::endl;
        exit(1);
    }

    return 0;
}