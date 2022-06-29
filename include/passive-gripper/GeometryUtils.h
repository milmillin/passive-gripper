#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "Constants.h"
#include "models/ContactPoint.h"
#include "models/MeshDependentResource.h"

namespace psg {

Fingers TransformFingers(const Fingers& fingers, const Eigen::Affine3d& trans);

inline auto TransformMatrix(const Eigen::MatrixXd& mat,
                            const Eigen::Affine3d& trans) {
  return (trans * mat.transpose().colwise().homogeneous()).transpose();
}

Eigen::Matrix<double, 8, 3> ComputeBoundingBox(const Fingers& fingers);

void AdaptiveSubdivideTrajectory(
    const Trajectory& trajectory,
    const Fingers& fingers,
    double flatness,
    Trajectory& out_trajectory,
    std::vector<std::pair<int, double>>& out_traj_contrib);

bool Remesh(const Eigen::MatrixXd& V,
            const Eigen::MatrixXi& F,
            size_t n_iters,
            Eigen::MatrixXd& out_V,
            Eigen::MatrixXi& out_F);

void Barycentric(const Eigen::Vector3d& p,
                 const Eigen::Vector3d& a,
                 const Eigen::Vector3d& b,
                 const Eigen::Vector3d& c,
                 double& out_u,
                 double& out_v,
                 double& out_w);

// points: N by dim matrix
bool ComputeConvexHull(const Eigen::MatrixXd& points,
                       std::vector<size_t>& out_hull_indices,
                       std::vector<std::vector<size_t>>& out_facets);

// Computes binormal B and tangential T given N.
// Assumes N is normalized
void GetPerp(const Eigen::Vector3d& N, Eigen::Vector3d& B, Eigen::Vector3d& T);

// Returns the double the area of triable ABC
double DoubleTriangleArea(const Eigen::Vector3d& A,
                          const Eigen::Vector3d& B,
                          const Eigen::Vector3d& C);

// a, b: angles in radians
// return difference between two angles wrapping considered
// (can be positive or negative)
double AngularDistance(double a, double b);

double SumSquaredAngularDistance(const Pose& a, const Pose& b);

// a, b: list of angles in radians
// Fix angle in b so that it takes the least distance
Pose FixAngles(const Pose& a, const Pose& b);

void FixTrajectory(Trajectory& t);

std::vector<ContactPoint> GenerateContactCone(const ContactPoint& contact_point,
                                              size_t cone_res,
                                              double friction);

std::vector<ContactPoint> GenerateContactCones(
    const std::vector<ContactPoint>& cps,
    size_t cone_res,
    double friction);

double Volume(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

Eigen::Vector3d CenterOfMass(const Eigen::MatrixXd& V,
                             const Eigen::MatrixXi& F);

Eigen::MatrixXd CreateCubeV(const Eigen::Vector3d& lb,
                            const Eigen::Vector3d& ub);

// par (>=0) vertex id, (-1) from, (-2) unreachable
void ComputeConnectivityFrom(const MeshDependentResource& mdr,
                             const Eigen::Vector3d& from,
                             std::vector<double>& out_dist,
                             std::vector<int>& out_par);

// Creates sphere meshes
// Input:
//  P:      #P by 3 coordinates of the centers of spheres
//  r:      radii of the spheres
//  res:    the resolution of the sphere discretization
// extendMesh if to extend the V,T,TC, or to overwrite them
// Output:
//  out_V:  #V by 3 sphere mesh coordinates
//  out_F:  #T by 3 sphere mesh triangles
// From
// https://github.com/avaxman/Directional/blob/master/include/directional/point_spheres.h
// Copyright (C) 2018 Amir Vaxman <avaxman@gmail.com>
// License: http://mozilla.org/MPL/2.0/
void CreateSpheres(const Eigen::MatrixXd& P,
                   double r,
                   int res,
                   Eigen::MatrixXd& out_V,
                   Eigen::MatrixXi& out_F);

// Creates cylinder mesh with axis of rotation on z-axis starting
// at o with height h in +z direction and radius r.
void CreateCylinderXY(const Eigen::Vector3d& o,
                      double r,
                      double h,
                      int res,
                      Eigen::MatrixXd& out_V,
                      Eigen::MatrixXi& out_F);

void CreateCylinder(const Eigen::Vector3d& a,
                    const Eigen::Vector3d& b,
                    double r,
                    int res,
                    Eigen::MatrixXd& out_V,
                    Eigen::MatrixXi& out_F);

void CreateCone(const Eigen::Vector3d& O,
                const Eigen::Vector3d& N,
                double r,
                double h,
                int res,
                Eigen::MatrixXd& out_V,
                Eigen::MatrixXi& out_F);

// Merge all disjoint sub-meshes in the mesh if they overlap
void MergeMesh(const Eigen::MatrixXd& V,
               const Eigen::MatrixXi& F,
               Eigen::MatrixXd& out_V,
               Eigen::MatrixXi& out_F);

// clang-format off
// Inline mesh of a cube
const Eigen::Matrix<double, 8, 3> cube_V = (Eigen::MatrixXd(8, 3) <<
  0.0, 0.0, 0.0,
  0.0, 0.0, 1.0,
  0.0, 1.0, 0.0,
  0.0, 1.0, 1.0,
  1.0, 0.0, 0.0,
  1.0, 0.0, 1.0,
  1.0, 1.0, 0.0,
  1.0, 1.0, 1.0).finished();
const Eigen::Matrix<int, 12, 3> cube_F = (Eigen::MatrixXi(12, 3) <<
  1, 7, 5,
  1, 3, 7,
  1, 4, 3,
  1, 2, 4,
  3, 8, 7,
  3, 4, 8,
  5, 7, 8,
  5, 8, 6,
  1, 5, 6,
  1, 6, 2,
  2, 6, 8,
  2, 8, 4).finished().array() - 1;
const Eigen::Matrix<int, 12, 2> cube_E = (Eigen::MatrixXi(12, 2) <<
  0, 1,
  1, 3,
  3, 2,
  2, 0,
  4, 5,
  5, 7,
  7, 6,
  6, 4,
  0, 4,
  1, 5,
  2, 6,
  3, 7).finished();

const Eigen::Matrix<double, 4, 3> axis_V = (Eigen::Matrix<double, 4, 3>() <<
  0, 0, 0,
  1, 0, 0,
  0, 1, 0,
  0, 0, 1).finished();

const Eigen::Matrix<int, 3, 2> axis_E = (Eigen::Matrix<int, 3, 2>() <<
  0, 1,
  0, 2,
  0, 3).finished();

const Eigen::Matrix<double, 4, 3> plane_V = (Eigen::Matrix<double, 4, 3>() <<
  -1, 0, -1,
  -1, 0, 1,
  1, 0, 1,
  1, 0, -1).finished();

const Eigen::Matrix<int, 2, 3> plane_F = (Eigen::Matrix<int, 2, 3>() <<
  0, 1, 2,
  0, 2, 3).finished();
// clang-format on

}  // namespace psg