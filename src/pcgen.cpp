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

#include <pcl/io/obj_io.h>
#include <pcl/point_cloud.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <TopoDS_Edge.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <STEPControl_StepModelType.hxx>
#include <STEPControl_Writer.hxx>

inline double uniform_deviate (int seed)
{
    double ran = seed * (1.0 / (RAND_MAX + 1.0));
    return ran;
}

inline void randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector4f& p)
{
    float r1 = static_cast<float> (uniform_deviate (rand ()));
    float r2 = static_cast<float> (uniform_deviate (rand ()));
    float r1sqr = std::sqrt (r1);
    float OneMinR1Sqr = (1 - r1sqr);
    float OneMinR2 = (1 - r2);
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
    p[3] = 0;
}

inline void randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p, bool calcNormal, Eigen::Vector3f& n)
{
    float r = static_cast<float> (uniform_deviate (rand ()) * totalArea);

    std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
    vtkIdType el = vtkIdType (low - cumulativeAreas->begin ());

    double A[3], B[3], C[3];
    vtkIdType npts = 0;
    vtkIdType *ptIds = nullptr;
    polydata->GetCellPoints (el, npts, ptIds);
    polydata->GetPoint (ptIds[0], A);
    polydata->GetPoint (ptIds[1], B);
    polydata->GetPoint (ptIds[2], C);
    if (calcNormal)
    {
        // OBJ: Vertices are stored in a counter-clockwise order by default
        Eigen::Vector3f v1 = Eigen::Vector3f (A[0], A[1], A[2]) - Eigen::Vector3f (C[0], C[1], C[2]);
        Eigen::Vector3f v2 = Eigen::Vector3f (B[0], B[1], B[2]) - Eigen::Vector3f (C[0], C[1], C[2]);
        n = v1.cross (v2);
        n.normalize ();
    }
    randomPointTriangle (float (A[0]), float (A[1]), float (A[2]),
                         float (B[0]), float (B[1]), float (B[2]),
                         float (C[0]), float (C[1]), float (C[2]), p);
}

void uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, bool calc_normal, pcl::PointCloud<pcl::PointNormal> & cloud_out)
{
    polydata->BuildCells ();
    vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();

    double p1[3], p2[3], p3[3], totalArea = 0;
    std::vector<double> cumulativeAreas (cells->GetNumberOfCells (), 0);
    size_t i = 0;
    vtkIdType npts = 0, *ptIds = nullptr;
    for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds); i++)
    {
        polydata->GetPoint (ptIds[0], p1);
        polydata->GetPoint (ptIds[1], p2);
        polydata->GetPoint (ptIds[2], p3);
        totalArea += vtkTriangle::TriangleArea (p1, p2, p3);
        cumulativeAreas[i] = totalArea;
    }

    cloud_out.points.resize (n_samples);
    cloud_out.width = n_samples;
    cloud_out.height = 1;

    for (i = 0; i < n_samples; i++)
    {
        Eigen::Vector4f p;
        Eigen::Vector3f n;
        randPSurface (polydata, &cumulativeAreas, totalArea, p, calc_normal, n);
        cloud_out.points[i].x = p[0];
        cloud_out.points[i].y = p[1];
        cloud_out.points[i].z = p[2];
        if (calc_normal)
        {
            cloud_out.points[i].normal_x = n[0];
            cloud_out.points[i].normal_y = n[1];
            cloud_out.points[i].normal_z = n[2];
        }
    }
}

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

                if(face.Orientation() == TopAbs_REVERSED) {
                    indexes.emplace_back(idx + 3, idx + 2, idx + 1);
                } else {
                    indexes.emplace_back(idx + 1, idx + 2, idx + 3);
                }

                idx += 3;
            }
        }
    }

    return std::pair(positions, indexes);
}

void WriteTempObjFile(
        const std::vector<Eigen::Vector3d>& vertices,
        const std::vector<Eigen::Vector3i>& indices,
        const std::string& filename = "temp.obj") {
    std::ofstream file;
    file.open(filename, std::ios::trunc);

    for (auto vertex : vertices) {
        file << "v " << vertex[0] << " " << vertex[1] << " " << vertex[2] << std::endl;
    }

    for (auto index : indices) {
        file << "f " << index[0] << " " << index[1] << " " << index[2] << std::endl;
    }

    file.close();
}

Eigen::Matrix4d buildTransformation(double tx, double ty, double tz, double rx, double ry, double rz){
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

void writePointCloud(const std::string& output, const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud, const Eigen::Matrix4d& transformation) {
    TopoDS_Compound source;
    TopoDS_Builder builder;
    builder.MakeCompound(source);

    for (auto & point : cloud->points) {
        Eigen::Vector4d vertex = {point.x, point.y, point.z, 1};
        Eigen::Vector4d normal = {point.normal_x, point.normal_y, point.normal_z, 0};
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

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
        vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New ();

        auto step = ReadShape(input.c_str());
        auto [vertices, indices] = Triangulate(step, d);
        WriteTempObjFile(vertices, indices);
        pcl::io::loadOBJFile("temp.obj", *mesh);
        pcl::io::mesh2vtk(*mesh, polydata);

        vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
        triangleFilter -> SetInputData(polydata);
        triangleFilter -> Update();

        vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        triangleMapper -> SetInputConnection (triangleFilter -> GetOutputPort());
        triangleMapper -> Update();
        polydata = triangleMapper -> GetInput();

        uniform_sampling(polydata, n, true, *cloud);
        auto transformation = buildTransformation(tx, ty, tz, rx, ry, rz);

        writePointCloud(output, cloud, transformation);

    } catch (const cxxopts::OptionException& e) {
        std::cerr << "Error parsing options: " << e.what() << std::endl;
        exit(1);
    }

    return 0;
}
