#include "QualityMetric.h"

#include <CGAL/QP_functions.h>
#include <CGAL/QP_models.h>
#include <utility>

#include "GeometryUtils.h"

#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

// #ifdef CGAL_USE_GMP
// #include <CGAL/Gmpzf.h>
// typedef CGAL::Gmpzf ET;
// #else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
// #endif

namespace psg {

static Eigen::MatrixXd CreateGraspMatrix(
    const std::vector<ContactPoint>& contact_cones,
    const Eigen::Vector3d& center_of_mass) {
  size_t nContacts = contact_cones.size();
  Eigen::MatrixXd G(6, nContacts);
  for (size_t i = 0; i < nContacts; i++) {
    G.block<3, 1>(0, i) = -contact_cones[i].normal;
    G.block<3, 1>(3, i) = (contact_cones[i].position - center_of_mass)
                              .cross(-contact_cones[i].normal);
  }
  return G;
}

static CGAL::Quotient<ET> MinNormVectorInFacet(const Eigen::MatrixXd& facet) {
  typedef CGAL::Quadratic_program<double> Program;
  typedef CGAL::Quadratic_program_solution<ET> Solution;

  size_t dim = facet.cols();

  Eigen::MatrixXd G;
  G = facet.transpose() * facet;
  G.diagonal().array() += kWrenchReg;
  G *= 2;

  // Solve QP to minimize x'Dx + c'x subject to Ax = B, x >= 0
  Program qp;

  // 1'x = 1
  for (size_t i = 0; i < dim; i++) {
    qp.set_a(i, 0, 1);
  }
  qp.set_b(0, 1);

  for (size_t i = 0; i < dim; i++) {
    for (size_t j = 0; j <= i; j++) {
      qp.set_d(i, j, G(i, j));
    }
  }

  Solution s = CGAL::solve_quadratic_program(qp, ET());
  return s.objective_value();
}

static CGAL::Quotient<ET> WrenchInPositiveSpan(
    const Eigen::MatrixXd& wrench_basis,
    const Eigen::VectorXd& target_wrench) {
  typedef CGAL::Quadratic_program<double> Program;
  typedef CGAL::Quadratic_program_solution<ET> Solution;

  // min (targetWrench - wrenchBasis * x)^2

  Eigen::MatrixXd D = wrench_basis.transpose() * wrench_basis;
  D.diagonal().array() += kWrenchReg;

  Eigen::VectorXd c = -wrench_basis.transpose() * target_wrench;

  // Solve QP to minimize x'Dx + c'x subject to Ax <= B, x >= 0
  Program qp(CGAL::SMALLER);

  // L1 finger contstraints
  /*
  size_t nWrenchesPerFinger = wrenchBasis.cols() / nFingers;
  for (size_t i = 0; i < nFingers; i++) {
    for (size_t j = 0; j < nWrenchesPerFinger; j++) {
      qp.set_a(i * nWrenchesPerFinger + j, i, 1);
    }
    qp.set_b(i, forceLimit);
  }
  */

  for (size_t i = 0; i < D.cols(); i++) {
    for (size_t j = 0; j <= i; j++) {
      qp.set_d(i, j, D(i, j));
    }
    qp.set_c(i, c(i));
  }

  Solution s = CGAL::solve_quadratic_program(qp, ET());
  /*
  for (auto it = s.variable_numerators_begin();
       it != s.variable_numerators_end();
       it++) {
    auto aa = *it;
    CGAL::to_double(aa);
  }
  */
  return s.objective_value() * 2 + target_wrench.squaredNorm();
}

bool CheckForceClosureQP(const std::vector<ContactPoint>& contact_cones,
                         const Eigen::Vector3d& center_of_mass) {
  Eigen::MatrixXd G = CreateGraspMatrix(contact_cones, center_of_mass);
  return MinNormVectorInFacet(G) < kWrenchNormThresh;
}

bool CheckPartialClosureQP(const std::vector<ContactPoint>& contact_cones,
                           const Eigen::Vector3d& center_of_mass,
                           const Eigen::Vector3d& ext_force,
                           const Eigen::Vector3d& ext_torque) {
  Eigen::MatrixXd G = CreateGraspMatrix(contact_cones, center_of_mass);
  Eigen::VectorXd targetWrench(6);
  targetWrench.block<3, 1>(0, 0) = -ext_force;
  targetWrench.block<3, 1>(3, 0) = -ext_torque;
  return WrenchInPositiveSpan(G, targetWrench) < kWrenchNormThresh;
}

double ComputeMinWrenchQP(const std::vector<ContactPoint>& contact_cones,
                          const Eigen::Vector3d& center_of_mass) {
  Eigen::MatrixXd G = CreateGraspMatrix(contact_cones, center_of_mass);
  if (MinNormVectorInFacet(G) >= kWrenchNormThresh) {
    // Zero not in convex hull
    return 0;
  }

  // Compute Convex Hull
  std::vector<size_t> hull_indices;
  std::vector<std::vector<size_t>> facets;
  if (ComputeConvexHull(G.transpose(), hull_indices, facets)) {
    CGAL::Quotient<ET> minDist;
    bool valid = false;
    // Compare against every facet
    for (const auto& facet : facets) {
      Eigen::MatrixXd F(6, facet.size());
      for (size_t i = 0; i < facet.size(); i++) {
        F.col(i) = G.col(facet[i]);
      }
      auto dist = MinNormVectorInFacet(F);
      if (!valid || dist < minDist) {
        minDist = dist;
        valid = true;
      }
    }
    if (!valid) std::cout << "Error: empty facet" << std::endl;
    return CGAL::to_double(minDist);
  }
  return 0;
}

double ComputePartialMinWrenchQP(const std::vector<ContactPoint>& contact_cones,
                                 const Eigen::Vector3d& center_of_mass,
                                 const Eigen::Vector3d& ext_force,
                                 const Eigen::Vector3d& ext_torque) {
  Eigen::MatrixXd G = CreateGraspMatrix(contact_cones, center_of_mass);
  Eigen::VectorXd target_wrench(6);
  target_wrench.block<3, 1>(0, 0) = -ext_force;
  target_wrench.block<3, 1>(3, 0) = -ext_torque;
  if (WrenchInPositiveSpan(G, target_wrench) >= kWrenchNormThresh) {
    // Not Partial Closure
    return 0.;
  }

  // Compute Convex Hull with Zero
  Eigen::MatrixXd V(G.cols() + 1, 6);
  V.block(1, 0, G.cols(), 6) = G.transpose();
  V.row(0).setZero();
  std::vector<size_t> hullIndices;
  std::vector<std::vector<size_t>> facets;
  if (ComputeConvexHull(V, hullIndices, facets)) {
    CGAL::Quotient<ET> minDist;
    bool valid = false;
    // Check against every face with Zero
    for (const auto& facet : facets) {
      bool zeroInFacet = false;
      for (size_t i : facet) {
        if (i == 0) {
          zeroInFacet = true;
          break;
        }
      }
      if (!zeroInFacet) continue;

      Eigen::MatrixXd F(6, facet.size() - 1);
      size_t id = 0;
      for (size_t i : facet) {
        if (i == 0) continue;
        F.col(id++) = G.col(i - 1);
      }
      auto dist = WrenchInPositiveSpan(F, target_wrench);
      if (!valid || dist < minDist) {
        minDist = dist;
        valid = true;
      }
    }
    if (valid)
      return CGAL::to_double(minDist);
    else
      return std::numeric_limits<double>::max();
  }
  return 0.0;
}

using autodiff::real;
using autodiff::Vector3real;

static real LossFn(const std::vector<Vector3real>& positions,
                   const std::vector<Vector3real>& normals,
                   Vector3real& trans,
                   Vector3real& rot,
                   Vector3real& center,
                   double maxCos) {
  assert(positions.size() == normals.size());
  size_t n = positions.size();

  real loss = 0;
  for (size_t i = 0; i < n; i++) {
    Vector3real v = trans + rot.cross(positions[i] - center);
    loss += std::max<real>(maxCos - v.dot(normals[i]), 0) +
            std::max<real>(v.norm() - 1, 0);
  }
  return loss;
}

bool CheckApproachDirection(const std::vector<ContactPoint>& contact_points,
                            Eigen::Affine3d& out_trans,
                            double max_angle,
                            double lr,
                            double threshold,
                            int max_iterations) {
  using autodiff::at;
  using autodiff::gradient;
  using autodiff::Matrix3real;
  using autodiff::wrt;

  Vector3real trans = Vector3real::Zero();
  Vector3real rot = Vector3real::Zero();
  Vector3real center = Vector3real::Zero();

  std::vector<Vector3real> positions;
  std::vector<Vector3real> normals;
  for (const auto& cp : contact_points) {
    positions.push_back(cp.position);
    normals.push_back(cp.normal.normalized());
  }

  double maxCos = std::cos(max_angle);

  real loss;
  for (int i = 0; i < max_iterations; ++i) {
    Eigen::VectorXd grad =
        gradient(LossFn,
                 wrt(trans, rot, center),
                 at(positions, normals, trans, rot, center, maxCos),
                 loss);
    grad *= lr;

    if (loss < threshold) {
      Eigen::Vector3d rotd = rot.cast<double>();
      Eigen::Vector3d transd = trans.cast<double>();
      Eigen::Vector3d centerd = center.cast<double>();
      double theta = rotd.norm();
      out_trans = Eigen::Translation3d(transd + centerd) *
                  Eigen::AngleAxisd(theta, rotd / theta) *
                  Eigen::Translation3d(-centerd);
      return true;
    }

    trans -= grad.block(0, 0, 3, 1);
    rot -= grad.block(3, 0, 3, 1);
    center -= grad.block(6, 0, 3, 1);
  }
  // std::cout << "failed: " << loss << std::endl;
  return false;
}

int GetFingerDistance(const DiscreteDistanceField& distance_field,
                      const std::vector<ContactPoint>& contact_points) {
  int max_distance = 0;
  for (auto& contact_point : contact_points) {
    max_distance =
        std::max(max_distance, distance_field.getVoxel(contact_point.position));
  }
  return max_distance;
}

double GetTrajectoryComplexity(const Trajectory& trajectory) {
  double sum = 0;
  for (size_t i = 1; i < trajectory.size(); i++) {
    sum += (trajectory[i] - trajectory[i - 1]).cwiseAbs().sum();
  }
  return sum;
}

}  // namespace psg