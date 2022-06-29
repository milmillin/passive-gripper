#include "Initialization.h"

#include <igl/random_points_on_mesh.h>
#include <random>
#include "DiscreteDistanceField.h"
#include "GeometryUtils.h"
#include "QualityMetric.h"
#include "robots/Robots.h"

#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include "../utils.h"

namespace psg {

// Distance from p to line ab
static bool PointToLineDist(const Eigen::Vector3d& a,
                            const Eigen::Vector3d& b,
                            const Eigen::Vector3d& p) {
  Eigen::Vector3d ab = b - a;
  Eigen::Vector3d ap = p - a;
  Eigen::Vector3d proj = (ap.dot(ab) / ab.squaredNorm()) * ab;
  return (ap - proj).norm();
}

static bool ShouldPopB(Eigen::Vector3d a,
                       Eigen::Vector3d c,
                       const MeshDependentResource& mdr) {
  Eigen::RowVector3d cc;
  double sign;
  Eigen::Vector3d ac = (c - a).normalized();
  a += ac * 1e-6;
  c -= ac * 1e-6;
  if (mdr.ComputeSignedDistance(a, cc, sign) < 0 ||
      mdr.ComputeSignedDistance(c, cc, sign) < 0)
    return false;

  igl::Hit hit;
  return !mdr.intersector.intersectSegment(
      a.transpose().cast<float>(), (c - a).transpose().cast<float>(), hit);
}

void InitializeMeshPosition(const Eigen::MatrixXd& V,
                            Eigen::MatrixXd& out_V,
                            Eigen::Affine3d& out_trans) {
  Eigen::Vector3d minimum = V.colwise().minCoeff();
  Eigen::Vector3d maximum = V.colwise().maxCoeff();

  Eigen::Vector3d translate(
      -minimum.x() / 2. - maximum.x() / 2., -minimum.y(), 0.07 - minimum.z());

  Eigen::MatrixXd SV = V.rowwise() + translate.transpose();
  Eigen::Translation3d mesh_trans(
      (SV.colwise().minCoeff() + SV.colwise().maxCoeff()) / 2.);

  Eigen::Affine3d trans = robots::Forward(kInitPose);
  SV = (trans * (SV.transpose().colwise().homogeneous())).transpose();

  double min_y = SV.colwise().minCoeff().y();
  SV.col(1).array() -= min_y;
  out_V = SV;
  out_trans = Eigen::Translation3d(0, -min_y, 0) * trans * mesh_trans;
}

Eigen::MatrixXd InitializeFinger(const ContactPoint contact_point,
                                 const MeshDependentResource& mdr,
                                 const Eigen::Vector3d& effector_pos,
                                 const std::vector<double>& dist,
                                 const std::vector<int>& par,
                                 size_t n_finger_joints) {
  Eigen::MatrixXd res(n_finger_joints, 3);
  Eigen::RowVector3d closest_point;

  int fid;
  mdr.ComputeClosestPoint(contact_point.position, closest_point, fid);

  // HACK
  closest_point += mdr.FN.row(fid) * 1e-5;

  size_t vid = -1;
  double bestDist = std::numeric_limits<double>::max();
  double curDist;
  for (int j = 0; j < 3; j++) {
    int v = mdr.F(fid, j);
    if ((curDist = (closest_point - mdr.V.row(v)).norm() + dist[v]) <
        bestDist) {
      bestDist = curDist;
      vid = v;
    }
  }
  std::vector<Eigen::Vector3d> finger;
  std::vector<int> fingerVid;
  finger.push_back(closest_point);
  fingerVid.push_back(-1);
  while (vid != -1) {
    Eigen::Vector3d toPush = mdr.V.row(vid);
    while (finger.size() > 1 &&
           ShouldPopB(finger[finger.size() - 2], toPush, mdr)) {
      finger.pop_back();
      fingerVid.pop_back();
    }
    finger.push_back(toPush);
    fingerVid.push_back(vid);
    vid = par[vid];
  }
  finger.push_back(effector_pos);
  fingerVid.push_back(-1);

  // Expand segment by 0.01 or half the clearance
  for (size_t j = 1; j < finger.size() - 1; j++) {
    Eigen::RowVector3d normal = mdr.VN.row(fingerVid[j]);
    igl::Hit hit;
    double avail_dis = 0.01;
    if (mdr.intersector.intersectRay(
            (mdr.V.row(fingerVid[j]) + normal * 1e-6).cast<float>(),
            normal.cast<float>(),
            hit)) {
      avail_dis = std::min(avail_dis, hit.t / 2.);
    }
    finger[j] += mdr.VN.row(fingerVid[j]) * avail_dis;
  }

  // Fix number of segment
  while (finger.size() > n_finger_joints) {
    size_t bestId = -1llu;
    size_t bestFallbackId = -1llu;
    double best = std::numeric_limits<double>::max();
    double bestFallback = std::numeric_limits<double>::max();
    for (size_t j = 1; j < finger.size() - 1; j++) {
      double cost = PointToLineDist(finger[j - 1], finger[j + 1], finger[j]);
      if (ShouldPopB(finger[j - 1], finger[j + 1], mdr)) {
        if (cost < best) {
          best = cost;
          bestId = j;
        }
      }
      if (cost < bestFallback) {
        bestFallback = cost;
        bestFallbackId = j;
      }
    }
    if (bestId == -1llu) bestId = bestFallbackId;
    finger.erase(finger.begin() + bestId);
  }
  while (finger.size() < n_finger_joints) {
    size_t bestId = -1llu;
    double best = -1;
    for (size_t j = 1; j < finger.size(); j++) {
      double cur = (finger[j] - finger[j - 1]).squaredNorm();
      if (cur > best) {
        best = cur;
        bestId = j;
      }
    }
    finger.insert(finger.begin() + bestId,
                  (finger[bestId] + finger[bestId - 1]) / 2.);
  }

  for (size_t j = 0; j < n_finger_joints; j++) {
    res.row(j) = finger[j];
  }
  return res;
}

std::vector<Eigen::MatrixXd> InitializeFingers(
    const std::vector<ContactPoint>& contact_points,
    const MeshDependentResource& mdr,
    const Eigen::Vector3d& effector_pos,
    size_t n_finger_joints) {
  std::vector<double> dist;
  std::vector<int> par;
  ComputeConnectivityFrom(mdr, effector_pos, dist, par);

  std::vector<Eigen::MatrixXd> res(contact_points.size());
  for (size_t i = 0; i < contact_points.size(); i++) {
    res[i] = InitializeFinger(
        contact_points[i], mdr, effector_pos, dist, par, n_finger_joints);
  }
  return res;
}

static void LengthParameterize(const Eigen::MatrixXd& V,
                               size_t nSteps,
                               Eigen::MatrixXd& out_V) {
  std::vector<double> cumDis(V.rows(), 0);
  for (size_t i = 1; i < V.rows(); i++) {
    cumDis[i] = cumDis[i - 1] + (V.row(i) - V.row(i - 1)).norm();
  }
  double disStep = cumDis.back() / nSteps;
  out_V.resize(nSteps + 1, 3);
  out_V.row(0) = V.row(0);
  double curDis = 0;
  size_t curV = 0;
  for (size_t i = 1; i < nSteps; i++) {
    double targetStep = i * disStep;
    while (curV + 1 < V.rows() && cumDis[curV + 1] < targetStep) {
      curV++;
    }
    double t = (targetStep - cumDis[curV]) / (cumDis[curV + 1] - cumDis[curV]);
    out_V.row(i) = V.row(curV + 1) * t + V.row(curV) * (1 - t);
  }
  out_V.row(nSteps) = V.row(V.rows() - 1);
}

Trajectory InitializeTrajectory0(const std::vector<Eigen::MatrixXd>& fingers,
                                 const Pose& initPose,
                                 size_t n_keyframes) {
  static constexpr size_t subdivide = 4;
  const size_t nSize = (n_keyframes - 1) * subdivide;
  Eigen::MatrixXd avgNorms(nSize, 3);
  avgNorms.setZero();
  Eigen::Affine3d fingerTransInv = robots::Forward(initPose).inverse();
  double minY = -0.05;
  for (size_t i = 0; i < fingers.size(); i++) {
    Eigen::MatrixXd finger;
    LengthParameterize(fingers[i], nSize, finger);
    avgNorms += (finger.block(1, 0, nSize, 3) - finger.block(0, 0, nSize, 3));
    Eigen::MatrixXd transformedFinger =
        (fingerTransInv * fingers[i].transpose().colwise().homogeneous())
            .transpose();
    minY = std::min(minY, transformedFinger.colwise().minCoeff()(1));
  }
  avgNorms /= fingers.size();
  Eigen::MatrixXd trans(n_keyframes, 3);
  trans.row(0).setZero();
  for (size_t i = 0; i < n_keyframes - 1; i++) {
    trans.row(i + 1) =
        trans.row(i) +
        avgNorms.block(i * subdivide, 0, subdivide, 3).colwise().sum();
  }

  Trajectory result;
  result.reserve(n_keyframes);
  result.push_back(initPose);
  Eigen::Affine3d cumTrans = robots::Forward(initPose);
  for (size_t i = 1; i < n_keyframes; i++) {
    cumTrans = Eigen::Translation3d(trans.row(i)) * cumTrans;
    Eigen::Affine3d curTrans = cumTrans;
    curTrans.translation().y() =
        std::max(curTrans.translation().y(), -minY + 0.003);
    std::vector<Pose> candidates;
    if (robots::Inverse(curTrans, candidates)) {
      double best = std::numeric_limits<double>::max();
      double cur;
      size_t bestI = -1;
      for (size_t j = 0; j < candidates.size(); j++) {
        if ((cur = SumSquaredAngularDistance(result.back(), candidates[j])) <
            best) {
          best = cur;
          bestI = j;
        }
      }
      result.push_back(FixAngles(result.back(), candidates[bestI]));
    }
  }
  return result;
}

Trajectory InitializeTrajectory1(const std::vector<Eigen::MatrixXd>& fingers,
                                 const Pose& init_pose,
                                 size_t n_keyframes) {
  Eigen::RowVector3d n(0, 0, 0);
  Eigen::RowVector3d eff_pos = robots::Forward(init_pose).translation();
  for (size_t i = 0; i < fingers.size(); i++) {
    n += (eff_pos - fingers[i].row(0));
  }
  n /= fingers.size();

  Eigen::Affine3d init_trans = robots::Forward(init_pose);
  Eigen::Affine3d finger_trans_inv = init_trans.inverse();
  double min_y = -0.05;
  for (size_t i = 0; i < fingers.size(); i++) {
    min_y = std::min(
        min_y,
        TransformMatrix(fingers[i], finger_trans_inv).colwise().minCoeff()(1));
  }
  Eigen::Affine3d end_trans = Eigen::Translation3d(n) * init_trans;
  end_trans.translation().y() =
      std::max(end_trans.translation().y(), -min_y + 0.003);

  Trajectory result;
  result.push_back(init_pose);
  result.push_back(init_pose);
  std::vector<Pose> candidates;
  size_t best_i;
  if (robots::BestInverse(end_trans, init_pose, candidates, best_i)) {
    Pose to_push = FixAngles(init_pose, candidates[best_i]);
    result.push_back(to_push);
    result.push_back(to_push);
  }
  return result;
}

Trajectory InitializeTrajectory(const std::vector<Eigen::MatrixXd>& fingers,
                                const Pose& initPose,
                                size_t n_keyframes) {
  return InitializeTrajectory1(fingers, initPose, n_keyframes);
}

void InitializeContactPointSeeds(const PassiveGripper& psg,
                                 size_t num_seeds,
                                 const ContactPointFilter& filter,
                                 std::vector<int>& out_FI,
                                 std::vector<Eigen::Vector3d>& out_X) {
  const MeshDependentResource& mdr_floor = psg.GetFloorMDR();
  const MeshDependentResource& mdr_remeshed = psg.GetRemeshedMDR();
  const MeshDependentResource& mdr = psg.GetMDR();
  double floor = psg.GetContactSettings().floor;

  Eigen::Vector3d effector_pos =
      robots::Forward(psg.GetParams().trajectory.front()).translation();

  std::vector<double> v_dist;
  std::vector<int> v_par;
  ComputeConnectivityFrom(mdr_floor, effector_pos, v_dist, v_par);

  Eigen::VectorXd K = mdr_remeshed.PV1.cwiseMax(mdr_remeshed.PV2);

  out_X.clear();
  out_FI.clear();

  double cos_angle = -cos(filter.angle);

  while (out_FI.size() < num_seeds) {
    Eigen::MatrixXd B_;
    Eigen::VectorXi out_FI_;
    Eigen::MatrixXd X_;
    igl::random_points_on_mesh(num_seeds, mdr.V, mdr.F, B_, out_FI_, X_);
    for (long long i = 0; i < X_.rows(); i++) {
      Eigen::RowVector3d x = X_.row(i);

      // filter floor
      if (x.y() <= floor) continue;
      // filter unreachable point
      int fid;
      Eigen::RowVector3d c;
      mdr_floor.ComputeClosestPoint(x, c, fid);
      if (abs((x - c).norm() - kExpandMesh) > 5e-4) continue;
      if (v_par[mdr_floor.F(fid, 0)] == -2)
        continue;  // check one vertex suffice

      /*
      // filter angle
      Eigen::RowVector3d n = mdr.FN.row(out_FI_(i));
      if (n.y() > cos_angle) continue;
      // filter hole
      igl::Hit hit;
      if (mdr.intersector.intersectRay(
              (x + n * 1e-6).cast<float>(), n.cast<float>(), hit)) {
        if (hit.t < filter.hole) continue;
      }
      // filter curvature
      Eigen::RowVector3d c;
      int fid;
      mdr_remeshed.ComputeClosestPoint(x, c, fid);
      Eigen::RowVector3i f = mdr_remeshed.F.row(fid);
      double u, v, w;
      Barycentric(c,
                  mdr_remeshed.V.row(f(0)),
                  mdr_remeshed.V.row(f(1)),
                  mdr_remeshed.V.row(f(2)),
                  u,
                  v,
                  w);
      double curvature = u * K(f(0)) + v * K(f(1)) + w * K(f(2));
      if (filter.curvature_radius * curvature > 1) continue;  // curvature > 1/r
      */

      out_FI.push_back(out_FI_(i));
      out_X.push_back(X_.row(i));
    }
  }
  Log() << "Num seeds: " << out_X.size() << std::endl;
}

std::vector<ContactPointMetric> InitializeContactPoints(
    const PassiveGripper& psg,
    const ContactPointFilter& filter,
    size_t num_candidates,
    size_t num_seeds) {
  const MeshDependentResource& mdr = psg.GetMDR();
  const ContactSettings& settings = psg.GetContactSettings();
  Eigen::Vector3d effector_pos =
      robots::Forward(psg.GetParams().trajectory.front()).translation();

  std::vector<int> FI;
  std::vector<Eigen::Vector3d> X;

  InitializeContactPointSeeds(psg, num_seeds, filter, FI, X);

  std::mt19937 gen;
  std::uniform_int_distribution<int> dist(0, num_seeds - 1);

  std::vector<ContactPointMetric> prelim;
  prelim.reserve(num_candidates);
  size_t total_iters = 0;

  // To be used for tolerance check
  Log() << "Building neighbor info" << std::endl;
  NeighborInfo neighborInfo(mdr.F);
  Log() << "Done building neighbor info" << std::endl;

  Log() << "Building distance field" << std::endl;
  DiscreteDistanceField distanceField(mdr.V, mdr.F, 50, effector_pos);
  Log() << "Done building distance field" << std::endl;
#pragma omp parallel
  {
    size_t iters = 0;
    while (true) {
      iters++;
      bool toContinue;
#pragma omp critical
      {
        toContinue = prelim.size() < num_candidates;
        if (iters >= 1000) {
          total_iters += iters;
          iters = 0;
          if (total_iters > num_candidates * 1000 &&
              total_iters > 10000 * prelim.size())  // success rate < 0.01%
            toContinue = false;
        }
      }
      if (!toContinue) break;

      // Random 3 contact points
      int pids[3] = {dist(gen), dist(gen), dist(gen)};
      if (pids[0] == pids[1] || pids[1] == pids[2] || pids[0] == pids[2])
        continue;
      std::vector<ContactPoint> contactPoints(3);
      std::vector<std::vector<int>> neighbors;
      for (int i = 0; i < 3; i++) {
        contactPoints[i].position = X[pids[i]];
        contactPoints[i].normal = mdr.FN.row(FI[pids[i]]);
        contactPoints[i].fid = FI[pids[i]];

        // Check tolerance
        neighbors.push_back(
            neighborInfo.GetNeighbors(contactPoints[i], mdr.V, mdr.F, 0.001));
      }

      // Check Feasibility: Minimum Wrench
      bool passMinimumWrench = true;
      std::vector<ContactPoint> contact_cones;
      double partialMinWrench = 0;
      int sample;
      for (sample = 0; sample < 1; sample++) {
        std::vector<ContactPoint> sample_contact_cones;
        sample_contact_cones.reserve(3 * settings.cone_res);
        for (size_t i = 0; i < 3; i++) {
          const auto&& cone = GenerateContactCone(
              contactPoints[i], settings.cone_res, settings.friction);
          sample_contact_cones.insert(
              sample_contact_cones.end(), cone.begin(), cone.end());
        }

        double samplePartialMinWrench =
            ComputePartialMinWrenchQP(sample_contact_cones,
                                      mdr.center_of_mass,
                                      -Eigen::Vector3d::UnitY(),
                                      Eigen::Vector3d::Zero());

        if (sample == 0) {
          contact_cones = sample_contact_cones;
          partialMinWrench = samplePartialMinWrench;
        }

        if (samplePartialMinWrench == 0) {
          passMinimumWrench = false;
          break;
        }

        // Prepare next sample
        for (size_t i = 0; i < 3; i++) {
          std::uniform_int_distribution<> dist(0, neighbors[i].size() - 1);
          int fid = neighbors[i][dist(gen)];
          contactPoints[i].normal = mdr.FN.row(fid);
        }
      }
      // Get at least a partial closure
      if (!passMinimumWrench) {
        // std::cout << "Failed after " << sample << " samples" << std::endl;
        continue;
      }

      for (int i = 0; i < 3; i++) {
        contactPoints[i].normal = mdr.FN.row(contactPoints[i].fid);
      }

      // Check Feasiblity: Approach Direction
      Eigen::Affine3d trans;
      if (!CheckApproachDirection(
              contactPoints, settings.max_angle, 1, 0.01, 1e-12, 500, trans)) {
        // std::cout << "Failed due to approach direction" << std::endl;
        continue;
      }
      // std::cout << "Success" << std::endl;
      double minWrench = ComputeMinWrenchQP(contact_cones, mdr.center_of_mass);

      ContactPointMetric candidate;
      candidate.contact_points = contactPoints;
      candidate.partial_min_wrench = partialMinWrench;
      candidate.min_wrench = minWrench;
      candidate.trans = trans;
      candidate.finger_distance =
          GetFingerDistance(distanceField, contactPoints);
#pragma omp critical
      {
        prelim.push_back(candidate);
        if (prelim.size() % 500 == 0)
          Log() << ">> prelim prog: " << prelim.size() << "/" << num_candidates
                << std::endl;
      }
    }
  }

  if (prelim.size() < num_candidates) {
    Error() << "low success rate. exit early. got: " << prelim.size()
            << " expected: " << num_candidates << std::endl;
  }

  std::sort(prelim.begin(),
            prelim.end(),
            [](const ContactPointMetric& a, const ContactPointMetric& b) {
              if (a.finger_distance == b.finger_distance)
                return a.partial_min_wrench > b.partial_min_wrench;
              return a.finger_distance < b.finger_distance;
            });

  // Remove solutions that are not in the frontier
  double partial_min_wrench = -1;
  std::vector<ContactPointMetric> result;
  for (int i = 0; i < prelim.size(); i++) {
    if (prelim[i].partial_min_wrench >= partial_min_wrench) {
      result.push_back(prelim[i]);
      partial_min_wrench = prelim[i].partial_min_wrench;
    }
  }

  return result;
}

void InitializeGripperBound(const PassiveGripper& psg,
                            Eigen::Vector3d& out_lb,
                            Eigen::Vector3d& out_ub) {
  out_lb.setZero();
  out_ub.setZero();

  double attachment_r = psg.GetTopoOptSettings().attachment_size / 2.;
  out_lb.x() = out_lb.y() = -attachment_r;
  out_ub.x() = out_ub.y() = attachment_r;

  Eigen::Affine3d finger_trans_inv = psg.GetFingerTransInv();
  for (const Eigen::MatrixXd& finger : psg.GetFingers()) {
    Eigen::MatrixXd transformedFinger =
        (finger_trans_inv * finger.transpose().colwise().homogeneous())
            .transpose();
    out_lb =
        out_lb.cwiseMin(transformedFinger.colwise().minCoeff().transpose());
    out_ub =
        out_ub.cwiseMax(transformedFinger.colwise().maxCoeff().transpose());
  }
  constexpr double padding = 0.03;
  out_lb.array() -= padding;
  out_ub.array() += padding;
  out_lb.z() = 0;
}

void InitializeConservativeBound(const PassiveGripper& psg,
                                 Eigen::Vector3d& out_lb,
                                 Eigen::Vector3d& out_ub) {
  out_lb.setZero();
  out_ub.setZero();

  double attachment_r = psg.GetTopoOptSettings().attachment_size / 2.;
  out_lb.x() = out_lb.y() = -attachment_r;
  out_ub.x() = out_ub.y() = attachment_r;

  Eigen::Affine3d finger_trans_inv = psg.GetFingerTransInv();

  auto V = (psg.GetFingerTransInv() *
            psg.GetMDR().V.transpose().colwise().homogeneous())
               .transpose();

  out_lb = out_lb.cwiseMin(V.colwise().minCoeff().transpose());
  out_ub = out_ub.cwiseMax(V.colwise().maxCoeff().transpose());

  constexpr double padding = 0.03;
  out_lb.array() -= padding;
  out_ub.array() += padding;
  out_lb.z() = 0;
}

}  // namespace psg