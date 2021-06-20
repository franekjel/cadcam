#pragma ide diagnostic ignored "cert-err58-cpp"
#include "src/cxxopts.h"
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRep_Builder.hxx>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <STEPControl_Writer.hxx>
#include <TopoDS_Builder.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <iostream>
#include <random>

std::mt19937 generator(std::random_device {}());
std::uniform_real_distribution<double> realDistribution(0.0, 1.0);
std::normal_distribution<double> normalDistribution(0.0, 1.0);

void generateModel(int n);
Eigen::Vector4d randomDisturbance(const Eigen::Vector4d v);
Eigen::Matrix4d randomTransformation();
std::pair<Eigen::Vector4d, Eigen::Vector4d> generate();
std::pair<Eigen::Vector4d, Eigen::Vector4d> bottomFace();
std::pair<Eigen::Vector4d, Eigen::Vector4d> rightFace();
std::pair<Eigen::Vector4d, Eigen::Vector4d> leftFace();
std::pair<Eigen::Vector4d, Eigen::Vector4d> frontFace();
std::pair<Eigen::Vector4d, Eigen::Vector4d> backFace();
std::pair<Eigen::Vector4d, Eigen::Vector4d> topFace();
std::pair<Eigen::Vector4d, Eigen::Vector4d> halfSphereFace();

int main(int argc, char** argv)
{
    cxxopts::Options options(argv[0], " - command line options");
    options.add_options()("n", "Number of points to generate", cxxopts::value<int>(), "points_num")("h,help", "Print help");

    try
    {
        auto result = options.parse(argc, argv);

        if (result.count("help"))
        {
            std::cout << options.help() << std::endl;
            exit(0);
        }

        if (result.count("n"))
        {
            int n = result["n"].as<int>();
            generateModel(n);
        }
    }
    catch (const cxxopts::OptionException& e)
    {
        std::cerr << "Error parsing options: " << e.what() << std::endl;
        exit(1);
    }

    return 0;
}

void generateModel(int n)
{
    auto transformation = randomTransformation();
    std::cout << "Transformation: " << std::endl
              << transformation << std::endl;
    TopoDS_Compound source;
    TopoDS_Builder builder;
    builder.MakeCompound(source);
    for (int i = 0; i < n; ++i)
    {
        Eigen::Vector4d vertex, normal;
        std::tie(vertex, normal) = generate();
        vertex = transformation * vertex;
        vertex.x() /= vertex.w();
        vertex.y() /= vertex.w();
        vertex.z() /= vertex.w();
        vertex = randomDisturbance(vertex);
        normal = transformation * normal;
        gp_Pnt P(vertex[0], vertex[1], vertex[2]);
        gp_Pnt N(normal[0], normal[1], normal[2]);
        TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(P, N);
        builder.Add(source, edge);
        std::cout << "Vertex: " << vertex.transpose() << std::endl;
        std::cout << "Normal: " << normal.transpose() << std::endl;
    }
    STEPControl_Writer writer;
    writer.Transfer(source, STEPControl_GeometricCurveSet);
    writer.Write("Output.stp");
}

Eigen::Vector4d randomDisturbance(const Eigen::Vector4d v)
{
    const double disturbance = 0.1;
    return Eigen::Vector4d(
        v.x() + (realDistribution(generator) - 0.5) * disturbance,
        v.y() + (realDistribution(generator) - 0.5) * disturbance,
        v.z() + (realDistribution(generator) - 0.5) * disturbance,
        1.0);
}

Eigen::Matrix4d randomTransformation()
{ // translation in y, no initial rotation
    double dx = 0;
    double dy = 0;
    double dz = 2.0;
    Eigen::Transform<double, 3, Eigen::Affine> transformation;
    transformation = Eigen::Translation<double, 3>(Eigen::Vector3d(dx, dy, dz));
    dx = 0.05 * M_PI; //realDistribution(generator) * M_PI;
    dy = 0.0 * M_PI;
    dz = 0.0 * M_PI; //realDistribution(generator) * M_PI;
    transformation.rotate(Eigen::AngleAxis<double>(dx, Eigen::Vector3d::UnitX()));
    transformation.rotate(Eigen::AngleAxis<double>(dy, Eigen::Vector3d::UnitY()));
    transformation.rotate(Eigen::AngleAxis<double>(dz, Eigen::Vector3d::UnitZ()));
    return transformation.matrix();
}

std::pair<Eigen::Vector4d, Eigen::Vector4d> generate()
{
    double p = realDistribution(generator) * (4 * 5 + 4 + M_PI);
    if (p < 4)
    {
        return bottomFace();
    }
    else if (p < 8)
    {
        return rightFace();
    }
    else if (p < 12)
    {
        return leftFace();
    }
    else if (p < 16)
    {
        return frontFace();
    }
    else if (p < 20)
    {
        return backFace();
    }
    else if (p < 20 + 4 - M_PI)
    {
        return topFace();
    }
    else
    {
        return halfSphereFace();
    }
}

std::pair<Eigen::Vector4d, Eigen::Vector4d> bottomFace()
{
    double x = realDistribution(generator) * 2 - 1;
    double y = -1;
    double z = realDistribution(generator) * 2 - 1;
    return std::make_pair(Eigen::Vector4d(x, y, z, 1), Eigen::Vector4d(0, -1, 0, 0));
}

std::pair<Eigen::Vector4d, Eigen::Vector4d> rightFace()
{
    double x = 1;
    double y = realDistribution(generator) * 2 - 1;
    double z = realDistribution(generator) * 2 - 1;
    return std::make_pair(Eigen::Vector4d(x, y, z, 1), Eigen::Vector4d(1, 0, 0, 0));
}

std::pair<Eigen::Vector4d, Eigen::Vector4d> leftFace()
{
    double x = -1;
    double y = realDistribution(generator) * 2 - 1;
    double z = realDistribution(generator) * 2 - 1;
    return std::make_pair(Eigen::Vector4d(x, y, z, 1), Eigen::Vector4d(-1, 0, 0, 0));
}

std::pair<Eigen::Vector4d, Eigen::Vector4d> frontFace()
{
    double x = realDistribution(generator) * 2 - 1;
    double y = realDistribution(generator) * 2 - 1;
    double z = 1;
    return std::make_pair(Eigen::Vector4d(x, y, z, 1), Eigen::Vector4d(0, 0, 1, 0));
}

std::pair<Eigen::Vector4d, Eigen::Vector4d> backFace()
{
    double x = realDistribution(generator) * 2 - 1;
    double y = realDistribution(generator) * 2 - 1;
    double z = -1;
    return std::make_pair(Eigen::Vector4d(x, y, z, 1), Eigen::Vector4d(0, 0, -1, 0));
}

std::pair<Eigen::Vector4d, Eigen::Vector4d> topFace()
{
    do
    {
        double x = realDistribution(generator) * 2 - 1;
        double y = 1;
        double z = realDistribution(generator) * 2 - 1;
        if (x * x + z * z > 1)
        { // 22% acceptance rate
            return std::make_pair(Eigen::Vector4d(x, y, z, 1), Eigen::Vector4d(0, 1, 0, 0));
        }
    } while (true);
}

std::pair<Eigen::Vector4d, Eigen::Vector4d> halfSphereFace()
{
    double x = normalDistribution(generator);
    double y = normalDistribution(generator);
    double z = normalDistribution(generator);
    double norm = sqrt(x * x + y * y + z * z);
    return std::make_pair(Eigen::Vector4d(x / norm, abs(y) / norm + 1, z / norm, 1),
        Eigen::Vector4d(x / norm, abs(y) / norm, z / norm, 0));
}
