// The `SweptVolume` and `SweptVolumeImpl` functions are adapted from
// the public code release of paper Swept Volumes via Spacetime Numerical
// Continuation. It comes with the following license:
//   Copyright (c) 2021 Silvia Sellán
//   SPDX-License-Identifier: MIT
//
// Other parts are licensed under:
//   Copyright (c) 2022 The University of Washington and Contributors
//   SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#include "SweptVolume.h"

#include <gradient_descent_test.h>
#include <igl/AABB.h>
#include <igl/WindingNumberAABB.h>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/copyleft/marching_cubes.h>
#include <igl/fast_winding_number.h>
#include <igl/random_points_on_mesh.h>
#include <sparse_continuation.h>
#include <cmath>
#include <vector>

#include "GeometryUtils.h"
#include "Initialization.h"
#include "robots/Robots.h"

namespace psg {

Eigen::Quaternion<double> logq(const Eigen::Quaternion<double>& q) {
  double exp_w = q.norm();
  double w = log(exp_w);
  double a = acos(q.w() / exp_w);

  if (a == 0.0) {
    return Eigen::Quaternion<double>(w, 0.0, 0.0, 0.0);
  }

  Eigen::Quaternion<double> res;
  res.w() = w;
  res.vec() = q.vec() / exp_w / (sin(a) / a);

  return res;
}

typedef std::function<
    double(double, const Eigen::RowVector3d&, const Eigen::RowVector3d&)>
    ImplicitMeshFunc;

static void SweptVolumeImpl(const Eigen::MatrixXd& V,
                            const Eigen::MatrixXi& F,
                            const std::vector<Eigen::Matrix4d>& Transformations,
                            ImplicitMeshFunc f,
                            Eigen::MatrixXd& out_V,
                            Eigen::MatrixXi& out_F,
                            bool flipped,
                            const double eps,
                            const int num_seeds) {
  const double iso = 0.0005;

  auto sgn = [](double val) -> double {
    return (double)((double(0) < val) - (val < double(0)));
  };

  Eigen::VectorXd time_keyframes;
  time_keyframes.setLinSpaced(Transformations.size(), 0.0, 1.0);
  double tau = time_keyframes(1);

  // Position function
  std::function<bool(const double,
                     Eigen::RowVector3d&,
                     Eigen::RowVector3d&,
                     Eigen::Matrix3d&,
                     Eigen::Matrix3d&)>
      interpolate_position = [&](const double t,
                                 Eigen::RowVector3d& xt,
                                 Eigen::RowVector3d& vt,
                                 Eigen::Matrix3d& Rt,
                                 Eigen::Matrix3d& VRt) -> bool {
    Eigen::RowVector3d x0, x1;
    int b;
    b = std::floor(t * (Transformations.size() - 1.0));
    if (t == 1.0) {
      b = b - 1;
    }
    double tt = (t - time_keyframes(b)) / (tau);

    // Milin: Use linear interpolation instead
    x0 = Transformations[b].block<3, 1>(0, 3).transpose();
    x1 = Transformations[b + 1].block<3, 1>(0, 3).transpose();
    xt = x0 * (1. - tt) + x1 * tt;
    vt = (x1 - x0) / tau;

    Eigen::Matrix3d R0, R1;
    R0 = Transformations[b].topLeftCorner(3, 3);
    R1 = Transformations[b + 1].topLeftCorner(3, 3);

    // scale
    Eigen::Matrix3d S0, S1, St, VSt;
    S0.setZero();
    S1.setZero();
    S0(0, 0) = R0.col(0).norm();
    S0(1, 1) = R0.col(1).norm();
    S0(2, 2) = R0.col(2).norm();
    S1(0, 0) = R1.col(0).norm();
    S1(1, 1) = R1.col(1).norm();
    S1(2, 2) = R1.col(2).norm();
    St = S1 + (1.0 - tt) * (S0 - S1);
    // rotation

    R0 = R0 * S0.inverse();
    R1 = R1 * S1.inverse();
    Eigen::Quaterniond q0(R0);

    Eigen::Quaterniond q1(R1);
    q1.normalize();
    q0.normalize();
    Eigen::Quaterniond qt = q0.slerp(tt, q1);
    Rt = qt.toRotationMatrix();
    if (q0.dot(q1) < 0) {
      q1.coeffs() = -q1.coeffs();
    }
    Eigen::Quaterniond qs = q0.conjugate() * q1;
    Eigen::Quaterniond qvt = qt * logq(qs);

    double qr, qi, qj, qk;
    Eigen::Matrix3d Rr, Ri, Rj, Rk;
    qr = qt.w();
    qi = qt.x();
    qj = qt.y();
    qk = qt.z();
    Rr << 0, -2 * qk, 2 * qj, 2 * qk, 0, -2 * qi, -2 * qj, 2 * qi, 0;
    Rk << -4 * qk, -2 * qr, 2 * qi, 2 * qr, -4 * qk, 2 * qj, 2 * qi, 2 * qj, 0;
    Rj << -4 * qj, 2 * qi, 2 * qr, 2 * qi, 0, 2 * qk, -2 * qr, 2 * qk, -4 * qj;
    Ri << 0, 2 * qj, 2 * qk, 2 * qj, -4 * qi, -2 * qr, 2 * qk, 2 * qr, -4 * qi;
    VRt = Rr * qvt.w() + Ri * qvt.x() + Rj * qvt.y() + Rk * qvt.z();
    VRt = VRt / tau;
    VSt = (S1 - S0) / tau;

    // Scaling
    Rt = Rt * St;
    VRt = VRt * St + Rt * VSt;
    return true;
  };

  igl::AABB<Eigen::MatrixXd, 3> tree;
  tree.init(V, F);
  igl::FastWindingNumberBVH fwn_bvh;
  int order = 2;
  igl::fast_winding_number(V, F, order, fwn_bvh);
  igl::WindingNumberAABB<Eigen::RowVector3d, Eigen::MatrixXd, Eigen::MatrixXi>
      hier;
  hier.set_mesh(V, F);
  hier.grow();

  // Signed Distance Evaluation Function
  int distance_queries = 0;
  int grad_descent_queries = 0;
  std::function<double(const Eigen::RowVector3d&,
                       double&,
                       std::vector<std::vector<double>>&,
                       std::vector<std::vector<double>>&,
                       std::vector<std::vector<double>>&)>
      scalarFunc = [&](const Eigen::RowVector3d& P,
                       double& time_seed,
                       std::vector<std::vector<double>>& intervals,
                       std::vector<std::vector<double>>& values,
                       std::vector<std::vector<double>>& minima) -> double {
    grad_descent_queries++;

    Eigen::RowVector3d running_closest_point = V.row(0);
    double running_sign = 1.0;

    std::function<double(const double)> f_ =
        [&, iso, P](const double t) -> double {
      int i;
      double s, sqrd, sqrd2, s2;
      Eigen::Matrix3d VRt, Rt;
      Eigen::RowVector3d xt, vt, pos, c, c2;
      interpolate_position(t, xt, vt, Rt, VRt);

      pos = ((Rt.inverse()) * ((P - xt).transpose())).transpose();
      // fast winding number
      Eigen::VectorXd w;
      igl::fast_winding_number(fwn_bvh, 2.0, pos, w);
      s = 1. - 2. * w(0);
      // running_sign = s;
      // double ub = (pos-running_closest_point) *
      // (pos-running_closest_point).transpose();
      sqrd = tree.squared_distance(V, F, pos, i, c);
      distance_queries = distance_queries + 1;
      // return sgn(s)*sqrt(sqrd) - 0.0;
      // return s*(c-pos).lpNorm<1>();

      double meshD = s * sqrt(sqrd) - iso;
      return f(meshD, P, pos);
      // return meshD;

      // return s*sqrt(sqrd) - iso;
      // return inigo_example(pos,0.0);
    };

    // Gradient of f
    std::function<double(const double)> gf = [&](const double t) -> double {
      int i;
      double s, sqrd, sqrd2, s2;
      Eigen::Matrix3d VRt, Rt;
      Eigen::RowVector3d xt, vt, pos, c, c2, point_velocity;
      //            xt = position(t);
      //            vt = velocity(t);
      //            Rt = rotation(t);
      //            VRt = rotational_velocity(t);
      interpolate_position(t, xt, vt, Rt, VRt);
      // pos = ((Rt.transpose())*((P - xt).transpose())).transpose();
      pos = ((Rt.inverse()) * ((P - xt).transpose())).transpose();
      // slow winding number
      // signed_distance_winding_number(tree,V,F,hier,pos,s,sqrd,i,c);
      // fast winding number
      Eigen::VectorXd w;
      igl::fast_winding_number(fwn_bvh, 2.0, pos, w);
      s = 1. - 2. * w(0);
      running_sign = s;
      // double ub = (pos-running_closest_point) *
      // (pos-running_closest_point).transpose();
      sqrd = tree.squared_distance(V, F, pos, i, c);
      //            if(running_sign>0){
      //                sqrd = tree.squared_distance(V,F,pos,0.0,ub,i,c);
      //            }else{
      //                sqrd = tree.squared_distance(V,F,pos,ub,10.0,i,c);
      //            }

      running_closest_point = c;

      Eigen::RowVector3d cp = c - pos;
      cp.normalize();
      point_velocity = (-Rt.inverse() * VRt * Rt.inverse() *
                            (P.transpose() - xt.transpose()) -
                        Rt.inverse() * vt.transpose())
                           .transpose();

      return (-s) * cp.dot(point_velocity);
      /*
      double meshD = s * sqrt(sqrd) - iso;
      double floorD = pos.y() - floor;
      double boxD = std::max((P.transpose() - boxUB).maxCoeff(),
                             (boxLB - P.transpose()).maxCoeff());
      double fD = std::max(boxD, -std::min(meshD, floorD));

      if (meshD == fD) {
        running_closest_point = c;

        Eigen::RowVector3d cp = c - pos;
        cp.normalize();
        point_velocity = (-Rt.inverse() * VRt * Rt.inverse() *
                              (P.transpose() - xt.transpose()) -
                          Rt.inverse() * vt.transpose())
                             .transpose();

        return (s) * cp.dot(point_velocity);
      } else if (floorD == fD) {
        running_closest_point = pos;
        running_closest_point.y() = floor;
        // Milin: this is actually -point_velocity?
        point_velocity = (-Rt.inverse() * VRt * Rt.inverse() *
                              (P.transpose() - xt.transpose()) -
                          Rt.inverse() * vt.transpose())
                             .transpose();
        return -point_velocity.y();
      } else {
        return 0;
      }
      */
    };

    // Run gradient descent
    double distance, seed;
    if (intervals.size() == 0) {
      std::vector<double> temp_interval;
      temp_interval.resize(0);
      intervals.push_back(temp_interval);
      values.push_back(temp_interval);
      minima.push_back(temp_interval);
    }
    gradient_descent_test(
        f_, gf, time_seed, distance, seed, intervals[0], values[0], minima[0]);
    time_seed = seed;  // updates seed so that we can add it to the queue in the
                       // next voxel
    return distance;
  };

  srand(100);
  // Initialization
  Eigen::MatrixXd X, N, B;
  Eigen::VectorXi I;
  igl::per_face_normals(V, F, Eigen::Vector3d(0.0, 0.0, -1.0).normalized(), N);
  igl::random_points_on_mesh(num_seeds, V, F, B, I, X);

  std::vector<double> init_times;
  init_times.push_back(0.0);
  std::vector<Eigen::RowVector3d> init_points;
  double minx, miny, minz;
  minx = 1000;
  miny = minx;
  minz = minx;
  Eigen::RowVector3d candidate;
  int counter = 0;
  init_times.push_back(0.0);
  double step = 0.5 / Transformations.size();
  for (int i = 0; i < X.rows(); i++) {
    Eigen::RowVector3d P = X.row(i);
    Eigen::Matrix3d VRt, Rt;
    Eigen::RowVector3d xt, vt, pos, c, c2, point_velocity, normal;
    for (double t = 0.0; t <= 1.0; t += step) {
      interpolate_position(t, xt, vt, Rt, VRt);

      pos = (Rt * P.transpose()).transpose() + xt;

      point_velocity = (VRt * P.transpose()).transpose() + vt;

      point_velocity.normalize();
      normal = (Rt * N.row(I(i)).transpose()).transpose();
      normal.normalize();
      // if ((fabs(normal.dot(point_velocity)) < 0.05) ||
      // (normal.dot(point_velocity) < 0.0 && t == 0.0) ||
      // (normal.dot(point_velocity) > 0.0 && t == 1.0)) {
      {
        candidate = pos + iso * normal;
        init_points.push_back(candidate);
        init_times.push_back(t);
        minx = std::min(minx, candidate(0));
        miny = std::min(miny, candidate(1));
        minz = std::min(minz, candidate(2));
      }
    }
  }

  std::vector<Eigen::RowVector3i> init_voxels;
  // init_points.push_back(Eigen::RowVector3d(0.0,0.0,1.0));
  // init_times.push_back(0.0);
  Eigen::RowVector3d p0;
  p0(0) = minx;
  p0(1) = miny;
  p0(2) = minz;
  init_voxels.resize(0);
  init_voxels.push_back(Eigen::RowVector3i(0, 0, 0));
  Eigen::RowVector3d this_point;
  int ix, iy, iz;
  for (int s = 0; s < init_points.size(); s++) {
    this_point = init_points[s];
    ix = std::floor((this_point[0] - minx) / eps);
    iy = std::floor((this_point[1] - miny) / eps);
    iz = std::floor((this_point[2] - minz) / eps);
    init_voxels.push_back(Eigen::RowVector3i(ix, iy, iz));
  }

  Eigen::MatrixXi CI;
  Eigen::MatrixXd CV;
  Eigen::VectorXd CS;
  Eigen::VectorXd CV_argmins;
  // CS: value, CV: vertex position, CI: indices
  sparse_continuation(p0,
                      init_voxels,
                      init_times,
                      scalarFunc,
                      eps,
                      1000000,
                      CS,
                      CV,
                      CI,
                      CV_argmins);
  if (flipped) CS *= -1.;
  igl::copyleft::marching_cubes(CS, CV, CI, 0., out_V, out_F);
}

static std::vector<Eigen::Matrix4d> GetTransformations(
    const PassiveGripper& psg) {
  static const size_t subdivision = 8;
  size_t n_steps = (psg.GetTrajectory().size() - 1) * subdivision + 1;
  size_t n_dof = psg.GetTrajectory().front().size();
  std::vector<Eigen::Matrix4d> transformations(n_steps);
  for (size_t i = 0; i < n_steps; i++) {
    size_t a = i / subdivision;
    size_t b = i % subdivision;
    if (i == n_steps - 1) {
      a--;
      b = subdivision;
    }

    double t = (double)b / subdivision;

    Pose l_pose;
    l_pose = psg.GetTrajectory()[a] * (1 - t) + psg.GetTrajectory()[a + 1] * t;
    Eigen::Affine3d finger_trans_inv =
        robots::Forward(psg.GetTrajectory().front()).inverse();
    transformations[i] =
        (finger_trans_inv * robots::Forward(l_pose)).inverse().matrix();
  }
  return transformations;
}

void SweptVolume(const Eigen::MatrixXd& V,
                 const Eigen::MatrixXi& F,
                 const std::vector<Eigen::Matrix4d>& transformations,
                 double res,
                 Eigen::MatrixXd& out_V,
                 Eigen::MatrixXi& out_F,
                 int num_seeds) {
  Eigen::MatrixXi CI;
  Eigen::MatrixXd CV;
  Eigen::VectorXd CS;
  ImplicitMeshFunc f = [](double sgd,
                          const Eigen::RowVector3d& P,
                          const Eigen::RowVector3d& pos) { return sgd; };
  SweptVolumeImpl(
      V, F, transformations, f, out_V, out_F, false, res, num_seeds);
}

void NegativeSweptVolume(const Eigen::MatrixXd& V,
                         const Eigen::MatrixXi& F,
                         const std::vector<Eigen::Matrix4d>& transformations,
                         const Eigen::Vector3d& box_lb,
                         const Eigen::Vector3d& box_ub,
                         const Eigen::Vector3d& floor,
                         const Eigen::Vector3d& floor_N,
                         double res,
                         Eigen::MatrixXd& out_V,
                         Eigen::MatrixXi& out_F,
                         const int num_seeds) {
  Eigen::Vector3d box_elb = box_lb.array() - res;
  Eigen::Vector3d box_eub = box_ub.array() + res;
  ImplicitMeshFunc f = [box_elb, box_eub, floor, floor_N](
                           double sgd,
                           const Eigen::RowVector3d& P,
                           const Eigen::RowVector3d& pos) {
    double floorD = (pos.transpose() - floor).dot(floor_N);
    double boxD = std::max((P.transpose() - box_eub).maxCoeff(),
                           (box_elb - P.transpose()).maxCoeff());
    return -std::max(boxD, -std::min(sgd, floorD));
  };

  Eigen::MatrixXd tmp_V;
  Eigen::MatrixXi tmp_F;
  SweptVolumeImpl(V, F, transformations, f, tmp_V, tmp_F, true, res, num_seeds);
  Eigen::MatrixXd box_V = CreateCubeV(box_lb, box_ub);
  igl::copyleft::cgal::mesh_boolean(tmp_V,
                                    tmp_F,
                                    box_V,
                                    cube_F,
                                    igl::MESH_BOOLEAN_TYPE_INTERSECT,
                                    out_V,
                                    out_F);
}

void NegativeSweptVolumePSG(const PassiveGripper& psg,
                            Eigen::MatrixXd& out_V,
                            Eigen::MatrixXi& out_F,
                            const int num_seeds) {
  Eigen::Vector3d floor(0, 0, 0);
  Eigen::Vector3d floor_N(0, 1, 0);
  Eigen::Affine3d finger_trans_inv = psg.GetFingerTransInv();
  floor = finger_trans_inv * floor;
  floor_N = finger_trans_inv.linear() * floor_N;

  NegativeSweptVolume(
      (finger_trans_inv * psg.GetMeshV().transpose().colwise().homogeneous())
          .transpose(),
      psg.GetMeshF(),
      GetTransformations(psg),
      psg.GetTopoOptSettings().lower_bound,
      psg.GetTopoOptSettings().upper_bound,
      floor,
      floor_N,
      psg.GetTopoOptSettings().neg_vol_res,
      out_V,
      out_F,
      num_seeds);
}

}  // namespace psg
