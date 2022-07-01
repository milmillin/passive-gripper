#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "Constants.h"
#include "models/ContactPoint.h"
#include "models/MeshDependentResource.h"

namespace psg {

/// <summary>
/// Transforms the fingers (aka skeleton)
/// </summary>
/// <param name="fingers">Input fingers</param>
/// <param name="trans">Transformation matrix</param>
/// <returns>Transformed fingers</returns>
Fingers TransformFingers(const Fingers& fingers, const Eigen::Affine3d& trans);

/// <summary>
/// Transform matrix
/// </summary>
/// <param name="mat">Input matrix</param>
/// <param name="trans">Transformation matrix</param>
/// <returns>Transformed matrix</returns>
inline auto TransformMatrix(const Eigen::MatrixXd& mat,
                            const Eigen::Affine3d& trans) {
  return (trans * mat.transpose().colwise().homogeneous()).transpose();
}

/// <summary>
/// Compute bounding box of the given fingers
/// </summary>
/// <param name="fingers">Fingers</param>
/// <returns>An 8-by-3 matrix containing the corners of the bounding
/// box</returns>
Eigen::Matrix<double, 8, 3> ComputeBoundingBox(const Fingers& fingers);

/// <summary>
/// Adaptively subdivide the given trajectory to a certain linearity tolerance.
/// The linear interpolation of the finger joints along the returned trajectory
/// will be within the tolerance to the actual positions.
/// </summary>
/// <param name="trajectory">Trajectory</param>
/// <param name="fingers">Fingers</param>
/// <param name="flatness">Linearity tolerance (m)</param>
/// <param name="out_trajectory">Output trajectory</param>
/// <param name="out_traj_contrib">Contribution of the original keyframes to the
/// returned keyframes</param>
void AdaptiveSubdivideTrajectory(
    const Trajectory& trajectory,
    const Fingers& fingers,
    double flatness,
    Trajectory& out_trajectory,
    std::vector<std::pair<int, double>>& out_traj_contrib);

/// <summary>
/// Remesh so that the edge length is about the same.
/// </summary>
/// <param name="V">Input mesh vertices</param>
/// <param name="F">Input mesh faces</param>
/// <param name="n_iters">Number of iterations</param>
/// <param name="out_V">Output mesh vertices</param>
/// <param name="out_F">Output mesh faces</param>
/// <returns>Whether the remeshing is successful</returns>
bool Remesh(const Eigen::MatrixXd& V,
            const Eigen::MatrixXi& F,
            size_t n_iters,
            Eigen::MatrixXd& out_V,
            Eigen::MatrixXi& out_F);

/// <summary>
/// Compute the barycentric coordinate of a given point given a triangle.
/// </summary>
/// <param name="p">The given point</param>
/// <param name="a">First vertex of the triangle</param>
/// <param name="b">Second vertex of the triangle</param>
/// <param name="c">Third vertex of the triangle</param>
/// <param name="out_u">Barycentric coordinate u</param>
/// <param name="out_v">Barycentric coordinate v</param>
/// <param name="out_w">Barycentric coordinate w</param>
void Barycentric(const Eigen::Vector3d& p,
                 const Eigen::Vector3d& a,
                 const Eigen::Vector3d& b,
                 const Eigen::Vector3d& c,
                 double& out_u,
                 double& out_v,
                 double& out_w);

/// <summary>
/// Compute the convex hull of a given points
/// </summary>
/// <param name="points">Points (N-by-dim matrix)</param>
/// <param name="out_hull_indices">Indices of points that belong to the convex
/// hull</param>
/// <param name="out_facets">List of faces on the convex hull</param>
/// <returns>Whether the computation is successful</returns>
bool ComputeConvexHull(const Eigen::MatrixXd& points,
                       std::vector<size_t>& out_hull_indices,
                       std::vector<std::vector<size_t>>& out_facets);

/// <summary>
/// Computes binormal B and tangential T given N. Assumes N is normalized.
/// </summary>
/// <param name="N">N</param>
/// <param name="B">B</param>
/// <param name="T">T</param>
void GetPerp(const Eigen::Vector3d& N, Eigen::Vector3d& B, Eigen::Vector3d& T);

/// <summary>
/// Compute the double area of the triangle ABC
/// </summary>
/// <param name="A">A</param>
/// <param name="B">B</param>
/// <param name="C">C</param>
/// <returns>The double area of the triangle</returns>
double DoubleTriangleArea(const Eigen::Vector3d& A,
                          const Eigen::Vector3d& B,
                          const Eigen::Vector3d& C);

/// <summary>
/// Compute the smallest difference between two angles a and b
/// (can be positive or negative)
/// </summary>
/// <param name="a">a (rad)</param>
/// <param name="b">b (rad)</param>
/// <returns>Angle in radians</returns>
double AngularDistance(double a, double b);

/// <summary>
/// Compute the sum squared angular distance between two poses a and b
/// </summary>
/// <param name="a">a</param>
/// <param name="b">b</param>
/// <returns>Sum squared angular distance</returns>
double SumSquaredAngularDistance(const Pose& a, const Pose& b);

/// <summary>
/// Fix angle in b so that it takes the least distance from a.
/// </summary>
/// <param name="a">Pose a (list of angles in rad)</param>
/// <param name="b">Pose b (list of angles in rad)</param>
/// <returns></returns>
Pose FixAngles(const Pose& a, const Pose& b);

/// <summary>
/// Call FixAngles on each keyframe except the first.
/// </summary>
/// <param name="t">Trajectory</param>
void FixTrajectory(Trajectory& t);

/// <summary>
/// Generate contact cones for a single contact point.
/// </summary>
/// <param name="contact_point">Contact point</param>
/// <param name="cone_res">Cone resolution</param>
/// <param name="friction">Friction</param>
/// <returns>List of ContactPoint (repurposed to represent the cone)</returns>
std::vector<ContactPoint> GenerateContactCone(const ContactPoint& contact_point,
                                              size_t cone_res,
                                              double friction);

/// <summary>
/// Generate contact cones for the given contact points.
/// </summary>
/// <param name="cps">Contact points</param>
/// <param name="cone_res">Cone resolution</param>
/// <param name="friction">Friction</param>
/// <returns>List of contact cones (size is cone_res * number of contact
/// points)</returns>
std::vector<ContactPoint> GenerateContactCones(
    const std::vector<ContactPoint>& cps,
    size_t cone_res,
    double friction);

/// <summary>
/// Calculate the volume of a mesh
/// </summary>
/// <param name="V">Input mesh vertices</param>
/// <param name="F">Input mesh faces</param>
/// <returns>The volume of a mesh</returns>
double Volume(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

/// <summary>
/// Calculate the center of mass of a mesh
/// </summary>
/// <param name="V">Input mesh vertices</param>
/// <param name="F">Input mesh faces</param>
/// <returns>The center of mass of a mesh</returns>
Eigen::Vector3d CenterOfMass(const Eigen::MatrixXd& V,
                             const Eigen::MatrixXi& F);

/// <summary>
/// Generate vertex positions of a cube given the lower and upper bound of the
/// cube.
/// </summary>
/// <param name="lb">Lower bound</param>
/// <param name="ub">Upper bound</param>
/// <returns>Cube vertices</returns>
Eigen::MatrixXd CreateCubeV(const Eigen::Vector3d& lb,
                            const Eigen::Vector3d& ub);

/// <summary>
/// Compute connectivity from a single point.
/// </summary>
/// <param name="mdr">Mesh dependent resources</param>
/// <param name="from">The point</param>
/// <param name="out_dist">Shortest edge distance from the point (size of
/// mdr.V)</param>
/// <param name="out_par">Parent of a vertex (>=0: vertex id, -1:
/// from, -2: unreachable)</param>
void ComputeConnectivityFrom(const MeshDependentResource& mdr,
                             const Eigen::Vector3d& from,
                             std::vector<double>& out_dist,
                             std::vector<int>& out_par);

/// <summary>
/// Create sphere meshes.
/// From
/// https://github.com/avaxman/Directional/blob/master/include/directional/point_spheres.h
/// Copyright (C) 2018 Amir Vaxman <avaxman@gmail.com>
/// License: http://mozilla.org/MPL/2.0/
/// </summary>
/// <param name="P">#P by 3 coordinate of the centers of spheres</param>
/// <param name="r">Radii of the spheres</param>
/// <param name="res">The resolution of the sphere discretization</param>
/// <param name="out_V">Spheres mesh vertices</param>
/// <param name="out_F">Spheres mesh faces</param>
void CreateSpheres(const Eigen::MatrixXd& P,
                   double r,
                   int res,
                   Eigen::MatrixXd& out_V,
                   Eigen::MatrixXi& out_F);

/// <summary>
/// Create a cylinder mesh with axis of rotation on z-axis starting
/// at o with height h in +z direction and radius r.
/// </summary>
/// <param name="o">o</param>
/// <param name="r">r</param>
/// <param name="h">h</param>
/// <param name="res">The resolution of the circle</param>
/// <param name="out_V">Cylinder mesh vertices</param>
/// <param name="out_F">Cylinder mesh faces</param>
void CreateCylinderXY(const Eigen::Vector3d& o,
                      double r,
                      double h,
                      int res,
                      Eigen::MatrixXd& out_V,
                      Eigen::MatrixXi& out_F);

/// <summary>
/// Create a cylinder mesh with the centers at a and b of radius r.
/// </summary>
/// <param name="a">a</param>
/// <param name="b">b</param>
/// <param name="r">r</param>
/// <param name="res">The resolution of the circle</param>
/// <param name="out_V">Cylinder mesh vertices</param>
/// <param name="out_F">Cylinder mesh faces</param>
void CreateCylinder(const Eigen::Vector3d& a,
                    const Eigen::Vector3d& b,
                    double r,
                    int res,
                    Eigen::MatrixXd& out_V,
                    Eigen::MatrixXi& out_F);

/// <summary>
/// Create a cone mesh with the center at O of radius r pointing with normal N
/// of height h.
/// </summary>
/// <param name="O">O</param>
/// <param name="N">N</param>
/// <param name="r">r</param>
/// <param name="h">h</param>
/// <param name="res">The resolution of the circle</param>
/// <param name="out_V">Cone mesh vertices</param>
/// <param name="out_F">Cone mesh faces</param>
void CreateCone(const Eigen::Vector3d& O,
                const Eigen::Vector3d& N,
                double r,
                double h,
                int res,
                Eigen::MatrixXd& out_V,
                Eigen::MatrixXi& out_F);

/// <summary>
/// Merge all disjoint sub-meshes if they overlap.
/// </summary>
/// <param name="V">Input mesh vertices</param>
/// <param name="F">Input mesh faces</param>
/// <param name="out_V">Merged mesh vertices</param>
/// <param name="out_F">Merged mesh faces</param>
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