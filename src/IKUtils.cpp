#include "IKUtils.h"
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

void robots::Forward(const std::vector<double>& jointConfig,
                     Eigen::Matrix3d& out_rot,
                     Eigen::Vector3d& out_trans) {
  if (jointConfig.size() != GetNumJoints()) {
    std::cout << "Error: forward kinematics expects 6 values" << std::endl;
    return;
  }

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

Eigen::Affine3d robots::Forward(const std::vector<double>& jointConfig) {
  Eigen::Matrix3d rot;
  Eigen::Vector3d trans;
  Forward(jointConfig, rot, trans);
  Eigen::Affine3d a;
  a.setIdentity();
  a.translate(trans);
  a.rotate(rot);
  return a;
}

bool robots::Inverse(const Eigen::Matrix3d& rot,
                     const Eigen::Vector3d& trans,
                     std::vector<std::vector<double>>& out_jointConfigs) {
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

  bool success = ComputeIk(eetrans,
                           eerot,
                           vfree.size() > 0 ? &vfree[0] : NULL,
                           solutions);

  if (!success) {
    std::cout << "Error: inverse kinematics failed" << std::endl;
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
    out_jointConfigs.push_back(solvalues);
  }

  return true;
}

bool robots::Inverse(const Eigen::Affine3d trans,
                     std::vector<std::vector<double>>& out_jointConfigs) {
  return Inverse(trans.linear(), trans.translation(), out_jointConfigs);
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

const double ur5_a[] = {0, -0.425, -0.39225, 0, 0, 0};
const double ur5_d[] = {0.089159, 0, 0, 0.10915, 0.09465, 0.0823};
const double ur5_alpha[] = {M_PI_2, 0, 0, M_PI_2, -M_PI_2, 0};
const size_t ur5_n_joints = 6;

void robots::ForwardIntermediate(const std::vector<double>& jointConfig,
                                  std::vector<Eigen::Affine3d>& out_trans) {
  out_trans.resize(ur5_n_joints);
  out_trans[0] =
      JointTransform(jointConfig[0], ur5_a[0], ur5_d[0], ur5_alpha[0]);
  for (size_t i = 1; i < ur5_n_joints; i++) {
    out_trans[i] =
        out_trans[i - 1] *
        JointTransform(jointConfig[i], ur5_a[i], ur5_d[i], ur5_alpha[i]);  
  }
}
