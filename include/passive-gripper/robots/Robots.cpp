#include "Robots.h"

#include "../GeometryUtils.h"

#define IKFAST_HAS_LIBRARY
#include <ikfast.h>

namespace psg {
namespace robots {
// Change axis
static const Eigen::Affine3d globalTrans =
    (Eigen::Affine3d)(Eigen::Matrix3d() << 1, 0, 0, 0, 0, 1, 0, -1, 0)
        .finished();
static const Eigen::Affine3d globalTransInv = globalTrans.inverse();

static void ForwardImpl(const Pose& jointConfig,
                        Eigen::Matrix3d& out_rot,
                        Eigen::Vector3d& out_trans) {
  double eerot[9];
  double eetrans[3];

  ComputeFk(&jointConfig[0], eetrans, eerot);

  for (size_t i = 0; i < 9; i++) {
    out_rot(i / 3, i % 3) = eerot[i];
  }
  for (size_t i = 0; i < 3; i++) {
    out_trans(i) = eetrans[i];
  }
}

Eigen::Affine3d Forward(const Pose& jointConfig) {
  Eigen::Matrix3d rot;
  Eigen::Vector3d trans;
  ForwardImpl(jointConfig, rot, trans);
  Eigen::Affine3d a;
  a.setIdentity();
  a.translate(trans);
  a.rotate(rot);
  return globalTrans * a;
}

static bool InverseImpl(const Eigen::Matrix3d& rot,
                        const Eigen::Vector3d& trans,
                        std::vector<Pose>& out_jointConfigs) {
  using namespace ikfast;
  IkSolutionList<IkReal> solutions;
  std::vector<IkReal> vfree(GetNumFreeParameters());

  double eerot[9];
  double eetrans[3];
  for (int i = 0; i < 9; i++) {
    eerot[i] = rot(i / 3, i % 3);
  }
  for (int i = 0; i < 3; i++) {
    eetrans[i] = trans(i);
  }

  bool success =
      ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

  if (!success) {
    return false;
  }

  size_t numSolutions = solutions.GetNumSolutions();
  std::vector<IkReal> solvalues(GetNumJoints());
  out_jointConfigs.clear();

  for (size_t i = 0; i < numSolutions; i++) {
    const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
    size_t numSolFreeParams = sol.GetFree().size();
    std::vector<IkReal> vsolfree(numSolFreeParams);
    sol.GetSolution(solvalues, vsolfree);
    Pose p;
    for (size_t j = 0; j < kNumDOFs; j++) {
      p[j] = solvalues[j];
    }
    out_jointConfigs.push_back(p);
  }

  return true;
}

bool Inverse(Eigen::Affine3d trans, std::vector<Pose>& out_jointConfigs) {
  trans = globalTransInv * trans;
  return InverseImpl(trans.linear(), trans.translation(), out_jointConfigs);
}

bool BestInverse(Eigen::Affine3d trans,
                 Pose base,
                 std::vector<Pose>& out_joint_configs,
                 size_t& best_i) {
  if (Inverse(trans, out_joint_configs)) {
    double best = std::numeric_limits<double>::max();
    double cur;
    size_t bestI = -1;
    for (size_t j = 0; j < out_joint_configs.size(); j++) {
      if ((cur = SumSquaredAngularDistance(base, out_joint_configs[j])) <
          best) {
        best = cur;
        bestI = j;
      }
    }
    best_i = bestI;
    return true;
  }
  best_i = -1;
  return false;
}

static Eigen::Affine3d JointTransform(double theta,
                                      double a,
                                      double d,
                                      double alpha) {
  Eigen::Affine3d result;
  result = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) *
           Eigen::Translation3d(a, 0, d) *
           Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX());
  return result;
}

void ForwardIntermediate(const Pose& jointConfig,
                         std::vector<Eigen::Affine3d>& out_trans) {
  out_trans.resize(kNumDOFs);
  out_trans[0] =
      JointTransform(jointConfig[0], kRobotA[0], kRobotD[0], kRobotAlpha[0]);
  for (size_t i = 1; i < kNumDOFs; i++) {
    out_trans[i] =
        out_trans[i - 1] *
        JointTransform(jointConfig[i], kRobotA[i], kRobotD[i], kRobotAlpha[i]);
  }
  for (size_t i = 0; i < kNumDOFs; i++) {
    out_trans[i] = globalTrans * out_trans[i];
  }
}

JacobianFunc ComputeJacobian(const Pose& jointConfig) {
  std::vector<Eigen::Affine3d> H(kNumDOFs + 1);
  std::vector<Eigen::Vector3d> Z(kNumDOFs);
  std::vector<Eigen::Vector3d> d(kNumDOFs);

  H[0].setIdentity();
  for (size_t i = 0; i < kNumDOFs; i++) {
    Z[i] = H[i].matrix().block<3, 1>(0, 2).transpose();
    d[i] = H[i].translation();
    H[i + 1] =
        H[i] *
        JointTransform(jointConfig(i), kRobotA[i], kRobotD[i], kRobotAlpha[i]);
  }

  Eigen::Affine3d Hlast = H.back();

  return [Z, d, Hlast](const Eigen::Vector3d& pos) {
    Jacobian J;
    Eigen::Vector3d pos_glob = Hlast * pos;
    for (size_t i = 0; i < kNumDOFs; i++) {
      J.block<3, 1>(0, i) = globalTrans * Z[i].cross(pos_glob - d[i]);
    }
    return J;
  };
}

}  // namespace robots
}  // namespace psg
