// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#include "Debugger.h"
#include "GeometryUtils.h"

namespace psg {

Debugger::Debugger() {
  Clear();
}

void Debugger::AddEdge(const Eigen::Vector3d& p0,
                       const Eigen::Vector3d& p1,
                       const Eigen::Vector3d& color) {
  std::lock_guard<std::mutex> guard(mtx_);
  if (n_P_ + 2 > P_.rows()) {
    P_.conservativeResize(P_.rows() + 10000, 3);
  }
  if (n_E_ + 1 > E_.rows()) {
    E_.conservativeResize(E_.rows() + 5000, 2);
  }
  if (n_C_ + 1 > C_.rows()) {
    C_.conservativeResize(C_.rows() + 5000, 3);
  }
  P_.row(n_P_) = p0;
  P_.row(n_P_ + 1) = p1;
  E_.row(n_E_) << n_P_, n_P_ + 1;
  C_.row(n_C_) = color;

  n_P_ += 2;
  n_E_++;
  n_C_++;
}

void Debugger::AddCube(const Eigen::Vector3d& org,
                       const Eigen::Vector3d& size) {
  std::lock_guard<std::mutex> guard(mtx_);
  if (n_V_ + 8 > V_.rows()) {
    V_.conservativeResize(V_.rows() + 800, 3);
  }
  if (n_F_ + 12 > F_.rows()) {
    F_.conservativeResize(F_.rows() + 1200, 3);
  }

  V_.block<8, 3>(n_V_, 0) =
      (cube_V.array().rowwise() * size.transpose().array()).rowwise() +
      org.transpose().array();
  F_.block<12, 3>(n_F_, 0) = cube_F.array() + n_V_;
  n_V_ += 8;
  n_F_ += 12;
}

void Debugger::AddPoint(const Eigen::Vector3d& p,
                        const Eigen::Vector3d& color) {
  std::lock_guard<std::mutex> guard(mtx_);
  if (n_PP_ + 1 > PP_.rows()) {
    PP_.conservativeResize(PP_.rows() + 500, 3);
  }
  if (n_PC_ + 1 > PC_.rows()) {
    PC_.conservativeResize(PC_.rows() + 500, 3);
  }

  PP_.block<1, 3>(n_PP_, 0) = p;
  PC_.block<1, 3>(n_PC_, 0) = color;
  n_PP_++;
  n_PC_++;
}

void Debugger::Clear() {
  P_.resize(1000, 3);
  E_.resize(500, 2);
  C_.resize(500, 3);
  V_.resize(800, 3);
  F_.resize(1200, 3);
  PP_.resize(500, 3);
  PC_.resize(500, 3);
  n_P_ = 0;
  n_E_ = 0;
  n_C_ = 0;
  n_V_ = 0;
  n_F_ = 0;
  n_PP_ = 0;
  n_PC_ = 0;
}

void Debugger::GetEdges(Eigen::MatrixXd& out_P,
                        Eigen::MatrixXi& out_E,
                        Eigen::MatrixXd& out_C) const {
  out_P = P_.block(0, 0, n_P_, 3);
  out_E = E_.block(0, 0, n_E_, 2);
  out_C = C_.block(0, 0, n_C_, 3);
}

void Debugger::GetMesh(Eigen::MatrixXd& out_V, Eigen::MatrixXi& out_E) const {
  out_V = V_.block(0, 0, n_V_, 3);
  out_E = F_.block(0, 0, n_F_, 3);
}

void Debugger::GetPoints(Eigen::MatrixXd& out_P, Eigen::MatrixXd& out_C) const {
  out_P = PP_.block(0, 0, n_PP_, 3);
  out_C = PC_.block(0, 0, n_PC_, 3);
}

}  // namespace psg