#pragma once
#include <Eigen/Core>

#include <TopoDS_Shape.hxx>

std::pair<Eigen::Matrix3Xf, Eigen::Matrix3Xf> ICP(const std::vector<TopoDS_Shape>& model, const std::vector<TopoDS_Shape>& points);
