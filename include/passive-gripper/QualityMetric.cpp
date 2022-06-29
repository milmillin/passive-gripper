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
    const std::vector<ContactPoint>& contactCones,
    const Eigen::Vector3d& centerOfMass) {
  size_t nContacts = contactCones.size();
  Eigen::MatrixXd G(6, nContacts);
  for (size_t i = 0; i < nContacts; i++) {
    G.block<3, 1>(0, i) = -contactCones[i].normal;
    G.block<3, 1>(3, i) = (contactCones[i].position - centerOfMass)
                              .cross(-contactCones[i].normal);
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
    const Eigen::MatrixXd& wrenchBasis,
    const Eigen::VectorXd& targetWrench) {
  typedef CGAL::Quadratic_program<double> Program;
  typedef CGAL::Quadratic_program_solution<ET> Solution;

  // min (targetWrench - wrenchBasis * x)^2

  Eigen::MatrixXd D = wrenchBasis.transpose() * wrenchBasis;
  D.diagonal().array() += kWrenchReg;

  Eigen::VectorXd c = -wrenchBasis.transpose() * targetWrench;

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
  return s.objective_value() * 2 + targetWrench.squaredNorm();
}

bool CheckForceClosureQP(const std::vector<ContactPoint>& contactCones,
                         const Eigen::Vector3d& centerOfMass) {
  Eigen::MatrixXd G = CreateGraspMatrix(contactCones, centerOfMass);
  return MinNormVectorInFacet(G) < kWrenchNormThresh;
}

bool CheckPartialClosureQP(const std::vector<ContactPoint>& contactCones,
                           const Eigen::Vector3d& centerOfMass,
                           const Eigen::Vector3d& extForce,
                           const Eigen::Vector3d& extTorque) {
  Eigen::MatrixXd G = CreateGraspMatrix(contactCones, centerOfMass);
  Eigen::VectorXd targetWrench(6);
  targetWrench.block<3, 1>(0, 0) = -extForce;
  targetWrench.block<3, 1>(3, 0) = -extTorque;
  return WrenchInPositiveSpan(G, targetWrench) < kWrenchNormThresh;
}

double ComputeMinWrenchQP(const std::vector<ContactPoint>& contactCones,
                          const Eigen::Vector3d& centerOfMass) {
  Eigen::MatrixXd G = CreateGraspMatrix(contactCones, centerOfMass);
  if (MinNormVectorInFacet(G) >= kWrenchNormThresh) {
    // Zero not in convex hull
    return 0;
  }

  // Compute Convex Hull
  std::vector<size_t> hullIndices;
  std::vector<std::vector<size_t>> facets;
  if (ComputeConvexHull(G.transpose(), hullIndices, facets)) {
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

double ComputePartialMinWrenchQP(const std::vector<ContactPoint>& contactCones,
                                 const Eigen::Vector3d& centerOfMass,
                                 const Eigen::Vector3d& extForce,
                                 const Eigen::Vector3d& extTorque) {
  Eigen::MatrixXd G = CreateGraspMatrix(contactCones, centerOfMass);
  Eigen::VectorXd targetWrench(6);
  targetWrench.block<3, 1>(0, 0) = -extForce;
  targetWrench.block<3, 1>(3, 0) = -extTorque;
  if (WrenchInPositiveSpan(G, targetWrench) >= kWrenchNormThresh) {
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
      auto dist = WrenchInPositiveSpan(F, targetWrench);
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

static real lossFn(const std::vector<Vector3real>& positions,
                   const std::vector<Vector3real>& normals,
                   Vector3real& trans,
                   Vector3real& rot,
                   Vector3real& center,
                   double maxCos,
                   double maxV) {
  assert(positions.size() == normals.size());
  auto n = positions.size();

  real loss = 0;
  for (size_t i = 0; i < n; i++) {
    Vector3real v = trans + rot.cross(positions[i] - center);
    // real x = std::max<real>(maxCos - v.normalized().dot(normals[i]), 0);
    // real y = std::max<real>(0.001 - v.norm(), 0);
    // loss += x * x + y * y;
    loss += std::max<real>(maxCos - v.dot(normals[i]), 0) +
            std::max<real>(v.norm() - maxV, 0);
  }
  return loss;
}

bool CheckApproachDirection(const std::vector<ContactPoint>& contactPoints,
                            double maxAngle,
                            double maxV,
                            double learningRate,
                            double threshold,
                            int max_iterations,
                            Eigen::Affine3d& out_trans) {
  using autodiff::at;
  using autodiff::gradient;
  using autodiff::Matrix3real;
  using autodiff::wrt;

  Vector3real trans = Vector3real::Zero();
  Vector3real rot = Vector3real::Zero();
  Vector3real center = Vector3real::Zero();

  std::vector<Vector3real> positions;
  std::vector<Vector3real> normals;
  for (const auto& cp : contactPoints) {
    positions.push_back(cp.position);
    normals.push_back(cp.normal.normalized());
  }

  double maxCos = std::cos(maxAngle);

  real loss;
  for (int i = 0; i < max_iterations; ++i) {
    Eigen::VectorXd grad =
        gradient(lossFn,
                 wrt(trans, rot, center),
                 at(positions, normals, trans, rot, center, maxCos, maxV),
                 loss);
    grad *= learningRate;

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

static real LossFn2(const std::vector<Vector3real>& positions,
                    const std::vector<Vector3real>& normals,
                    Vector3real& trans,
                    Vector3real& rot,
                    Vector3real& center,
                    real away_dist) {
  assert(positions.size() == normals.size());
  size_t n = positions.size();

  real loss = 0;
  for (size_t i = 0; i < n; i++) {
    Vector3real v = trans + rot.cross(positions[i] - center);
    // real x = std::max<real>(away_dist - v.norm(), 0);
    // real y = std::max<real>(max_cos - v.normalized().dot(normals[i]), 0);
    // real y = std::max<real>(v.norm() - limit, 0);
    real x = std::max<real>(away_dist - v.dot(normals[i]), 0);
    loss += x * x;
  }
  return loss;
}

bool CheckApproachDirection2(const std::vector<ContactPoint>& contact_points,
                             double away_dist,
                             double max_angle,
                             const Eigen::Vector3d& center_of_mass,
                             Eigen::Affine3d& out_trans) {
  using autodiff::at;
  using autodiff::gradient;
  using autodiff::Matrix3real;
  using autodiff::wrt;

  constexpr double learningRate = 0.1;

  Vector3real trans = Vector3real::Zero();
  for (size_t i = 0; i < contact_points.size(); i++) {
    trans += contact_points[i].normal;
  }
  trans /= contact_points.size();

  Vector3real rot = Vector3real::Zero();
  Vector3real center = center_of_mass.cast<real>();

  std::vector<Vector3real> positions;
  std::vector<Vector3real> normals;
  for (const auto& cp : contact_points) {
    positions.push_back(cp.position);
    normals.push_back(cp.normal.normalized());
    trans += cp.normal;
  }
  trans /= contact_points.size();

  double max_cos = cos(max_angle);

  real loss;
  for (int i = 0; i < 10000; ++i) {
    Eigen::VectorXd grad =
        gradient(LossFn2,
                 wrt(trans, rot, center),
                 at(positions, normals, trans, rot, center, away_dist),
                 loss);
    grad *= learningRate;

    if (loss < 1e-12) {
      // std::cout << i << " loss: " << loss << std::endl;

      Eigen::Vector3d rotd = rot.cast<double>();
      Eigen::Vector3d transd = trans.cast<double>();
      Eigen::Vector3d centerd = center.cast<double>();

      double maxv = 0;
      for (size_t j = 0; j < positions.size(); j++) {
        Eigen::Vector3d v =
            transd + rotd.cross(contact_points[j].position - centerd);
        maxv = std::max(v.norm(), maxv);
      }

      // std::cout << maxv << std::endl;

      double factor = away_dist / maxv;

      transd *= factor;
      rotd *= factor;

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

struct pair_hash {
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2>& pair) const {
    return (std::hash<T1>()(pair.first) << 1) ^ std::hash<T2>()(pair.second);
  }
};

static inline std::pair<int, int> makeEdge(int a, int b) {
  if (a < b) {
    return std::pair<int, int>(a, b);
  } else {
    return std::pair<int, int>(b, a);
  }
}

NeighborInfo::NeighborInfo(const Eigen::MatrixXi& F) {
  std::unordered_map<std::pair<int, int>, std::unordered_set<int>, pair_hash>
      edge_to_face;
  for (int row = 0; row < F.rows(); row++) {
    edge_to_face[makeEdge(F(row, 0), F(row, 1))].insert(row);
    edge_to_face[makeEdge(F(row, 1), F(row, 2))].insert(row);
    edge_to_face[makeEdge(F(row, 2), F(row, 0))].insert(row);
  }

  neighbor.clear();
  for (const auto& item : edge_to_face) {
    for (int face1 : item.second) {
      for (int face2 : item.second) {
        if (face1 != face2) {
          neighbor[face1].insert(face2);
          neighbor[face2].insert(face1);
        }
      }
    }
  }
}

std::vector<int> NeighborInfo::GetNeighbors(const ContactPoint& contact_point,
                                            const Eigen::MatrixXd& V,
                                            const Eigen::MatrixXi& F,
                                            double tolerance) const {
  std::unordered_set<int> visited;
  std::vector<int> current;
  std::vector<int> result;
  current.push_back(contact_point.fid);
  visited.insert(contact_point.fid);
  result.push_back(contact_point.fid);
  tolerance = tolerance * tolerance;
  while (current.size() > 0) {
    int face = *current.rbegin();
    current.pop_back();
    if (visited.find(face) != visited.end()) {
      continue;
    }

    bool valid = false;
    for (int i = 0; i < 3; i++) {
      valid = valid || ((V.row(F(face, i)).transpose() - contact_point.position)
                            .squaredNorm() < tolerance);
    }

    result.push_back(face);

    for (int next : neighbor.at(face)) {
      if (visited.find(next) == visited.end()) {
        visited.insert(next);
        current.push_back(next);
      }
    }
  }
  return result;
}

int GetFingerDistance(const DiscreteDistanceField& distanceField,
                      const std::vector<ContactPoint>& contact_points) {
  int max_distance = 0;
  for (auto& contact_point : contact_points) {
    // std::cout << distanceField.getVoxel(contact_point.position) << std::endl;
    max_distance =
        std::max(max_distance, distanceField.getVoxel(contact_point.position));
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