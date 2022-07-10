#include "CostFunctions.h"

#include <igl/copyleft/cgal/intersect_other.h>
#include "GeometryUtils.h"
#include "robots/Robots.h"

namespace psg {

// See CHOMP paper page 4
static double PotentialSDF(double s) {
  static constexpr double epsilon = 0.001;
  if (s > epsilon) return 0;
  if (s < 0) return -s + (epsilon / 2.);
  double tmp = s - epsilon;
  return tmp * tmp / (2. * epsilon);
}

static double GetDist(const Eigen::Vector3d& p,
                      const CostSettings& settings,
                      const MeshDependentResource& mdr) {
  Eigen::RowVector3d c;
  double sign;
  double s = mdr.ComputeSignedDistance(p, c, sign);
  double sFloor = p.y() - settings.floor;
  if (s < sFloor) return s;
  return sFloor;
}

double EvalAt(const Eigen::Vector3d& p,
              const CostSettings& settings,
              const MeshDependentResource& mdr) {
  double result = PotentialSDF(GetDist(p, settings, mdr));
  return result;
}

double MinDistance(const GripperParams& params,
                   const GripperSettings& settings,
                   const MeshDependentResource& mdr) {
  constexpr double precision = 0.001;  // 1mm

  Eigen::Affine3d finger_trans_inv =
      robots::Forward(params.trajectory.front()).inverse();
  std::vector<Eigen::MatrixXd> fingers =
      TransformFingers(params.fingers, finger_trans_inv);

  // discretize fingers
  std::vector<Eigen::Vector3d> d_fingers;
  for (size_t i = 0; i < fingers.size(); i++) {
    for (Eigen::Index j = 1; j < fingers[i].rows(); j++) {
      double norm = (fingers[i].row(j) - fingers[i].row(j - 1)).norm();
      size_t subs = std::ceil(norm / precision);
      for (size_t k = (j == 1) ? 0 : 1; k <= subs; k++) {
        double t = (double)k / subs;
        d_fingers.push_back(fingers[i].row(j - 1) * (1. - t) +
                            fingers[i].row(j) * t);
      }
    }
  }
  Eigen::MatrixXd D_fingers(d_fingers.size(), 3);
#pragma omp parallel for
  for (size_t i = 0; i < d_fingers.size(); i++) {
    D_fingers.row(i) = d_fingers[i];
  }

  // discretize time
  Trajectory new_trajectory;
  std::vector<std::pair<int, double>> traj_contrib;
  AdaptiveSubdivideTrajectory(params.trajectory,
                              params.fingers,
                              precision,
                              new_trajectory,
                              traj_contrib);
  size_t n_trajectory = new_trajectory.size();
  std::vector<Fingers> new_fingers(n_trajectory);
  for (size_t i = 0; i < n_trajectory; i++) {
    new_fingers[i] =
        TransformFingers(fingers, robots::Forward(new_trajectory[i]));
  }

  double min_dist = 0;

  for (size_t i = 0; i < n_trajectory - 1; i++) {
    double max_deviation = 0;
    for (size_t j = 0; j < new_fingers[i].size(); j++) {
      double dev = (new_fingers[i + 1][j] - new_fingers[i][j])
                       .rowwise()
                       .norm()
                       .maxCoeff();
      max_deviation = std::max(max_deviation, dev);
    }
    size_t cur_sub = std::max<size_t>(std::ceil(max_deviation / precision), 1);

    size_t iters = cur_sub;
    if (i == n_trajectory - 2) iters++;

    for (size_t j = 0; j < iters; j++) {
      double t = (double)j / cur_sub;
      Pose pose = new_trajectory[i] * (1. - t) + new_trajectory[i + 1] * t;
      Eigen::MatrixXd f = TransformMatrix(D_fingers, robots::Forward(pose));
#pragma omp parallel
      {
        double t_min = 0;
#pragma omp for nowait
        for (long long k = 0; k < f.rows(); k++) {
          t_min = std::min(t_min, GetDist(f.row(k), settings.cost, mdr));
        }
#pragma omp critical
        min_dist = std::min(min_dist, t_min);
      }
    }
  }
  return min_dist;
}

struct _SegState {
  bool is_first;
  bool is_in;
  size_t last_pos_vid;
  double last_pos_vid_dis;
  Eigen::Vector3d last_pos;  // last intersection entering the mesh
};

#define soft_assert(x) \
  if (!(x)) fprintf(stderr, "Assertion Error: " #x "" __FILE__ ":%d", __LINE__)

static double ComputeCollisionPenaltySegment(const Eigen::Vector3d& A,
                                             const Eigen::Vector3d& B,
                                             const MeshDependentResource& mdr,
                                             double geodesic_contrib,
                                             double inner_dis_contrib,
                                             _SegState& state,
                                             Debugger* const debugger,
                                             const Eigen::RowVector3d& color) {
  Eigen::RowVector3d dir = B - A;
  double norm = dir.norm();
  if (norm < 1e-12 || isnan(norm)) return 0;
  dir /= norm;
  // std::cout << dir << std::endl;
  std::vector<igl::Hit> hits;
  int num_rays;

  mdr.intersector.intersectRay(
      A.cast<float>(),
      dir.cast<float>(),
      hits,
      num_rays,
      0,
      state.is_first ? std::numeric_limits<float>::max() : norm + 1e-6);

  if (state.is_first) {
    state.is_in = hits.size() % 2 == 1;
  }

  Eigen::RowVector3d color_inv = Eigen::RowVector3d::Ones() - color;

  Eigen::Vector3d last_hit = A;

  double total_dis = 0;
  for (const auto& hit : hits) {
    if (hit.t >= norm) break;
    Eigen::RowVector3d P = A.transpose() + dir * hit.t;

    // find closest vertex for P
    size_t vid = -1;
    double best_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < 3; i++) {
      int u = mdr.F(hit.id, i);
      double d = (P - mdr.V.row(u)).squaredNorm();
      if (d < best_dist) {
        best_dist = d;
        vid = u;
      }
    }
    best_dist = sqrt(best_dist);

    if (state.is_in) {
      // --|P
      if (inner_dis_contrib != 0.) {
        total_dis += (P - last_hit.transpose()).norm() * inner_dis_contrib;
        if (debugger) {
          debugger->AddEdge(last_hit, P.transpose(), color);
        }
      }

      if (!state.is_first) {
        // state.last_pos |----| P
        soft_assert(state.last_pos_vid != -1llu);

        if (geodesic_contrib != 0.) {
          const Eigen::MatrixXd& SP_ = mdr.GetSP();
          const Eigen::MatrixXi& SP_par_ = mdr.GetSPPar();
          total_dis += (state.last_pos_vid_dis + SP_(state.last_pos_vid, vid) +
                        best_dist) *
                       geodesic_contrib;
          if (debugger) {
            debugger->AddEdge(
                state.last_pos, mdr.V.row(state.last_pos_vid), colors::kRed);
            debugger->AddEdge(mdr.V.row(vid), P, colors::kBrown);
            size_t cur = vid;
            while (cur != state.last_pos_vid) {
              size_t par = SP_par_(cur, state.last_pos_vid);
              debugger->AddEdge(
                  mdr.V.row(par), mdr.V.row(cur), colors::kPurple);
              cur = par;
            }
          }
        }
      }
      state.last_pos_vid = -1llu;
    } else {
      // P|--
      state.last_pos = P;
      state.last_pos_vid = vid;
      state.last_pos_vid_dis = best_dist;

      if (debugger) {
        debugger->AddEdge(last_hit, P.transpose(), color_inv);
      }
    }
    state.is_in = !state.is_in;
    state.is_first = false;
    last_hit = P;
  }

  if (state.is_in) {
    // |-- B
    if (inner_dis_contrib != 0.) {
      total_dis += (B - last_hit).norm() * inner_dis_contrib;
      if (debugger) {
        debugger->AddEdge(B, last_hit, color);
      }
    }
  } else {
    if (debugger) {
      debugger->AddEdge(B, last_hit, color_inv);
    }
  }
  return total_dis;
}

double ComputeFloorCost(Eigen::RowVector3d p0,
                        Eigen::RowVector3d p1,
                        double floor) {
  if (p0.y() >= floor && p1.y() >= floor) return 0;
  Eigen::RowVector3d p01 = p1 - p0;
  if ((p0.y() < floor) != (p1.y() < floor)) {
    if (p1.y() < floor) std::swap(p0, p1);
    p01 *= (floor - p0.y()) / (p1.y() - p0.y());
  }
  double cost = p01.norm();
  p01.y() = 0;
  return cost + p01.norm();
}

double ComputeCost_SP(const GripperParams& params,
                      const GripperParams& init_params,
                      const GripperSettings& settings,
                      const MeshDependentResource& remeshed_mdr,
                      const CostContext& context) {
  struct _SubInfo {
    // Pose pose;
    Fingers fingers;
    std::vector<Eigen::VectorXd> costs;
  };

  if (params.fingers.empty()) return 0;

  Eigen::Affine3d finger_trans_inv =
      robots::Forward(params.trajectory.front()).inverse();
  std::vector<Eigen::MatrixXd> fingers =
      TransformFingers(params.fingers, finger_trans_inv);

  // Cost between two points
  const double floor = settings.cost.floor;
  const double inner_dis_contrib = settings.cost.inner_dis_contrib;
  const double geodesic_contrib = settings.cost.geodesic_contrib;
  auto MyCost =
      [floor, &context, inner_dis_contrib, geodesic_contrib, &remeshed_mdr](
          const Eigen::RowVector3d& p0,
          const Eigen::RowVector3d& p1,
          _SegState& state,
          const Eigen::RowVector3d& color) -> double {
    return ComputeCollisionPenaltySegment(p0,
                                          p1,
                                          remeshed_mdr,
                                          geodesic_contrib,
                                          inner_dis_contrib,
                                          state,
                                          context.debugger,
                                          color) +
           ComputeFloorCost(p0, p1, floor);
  };

  // Gripper energy at particular time
  auto ProcessFinger = [&MyCost](const Fingers& fingers) -> double {
    double cost = 0;
    _SegState state;
    for (size_t i = 0; i < fingers.size(); i++) {
      state.is_first = true;
      for (Eigen::Index j = 0; j < fingers[i].rows() - 1; j++) {
        cost += MyCost(fingers[i].row(j),
                       fingers[i].row(j + 1),
                       state,
                       Eigen::RowVector3d(1, 0.5, 0));
      }
    }
    return cost;
  };

  // Linearize trajectory
  Trajectory new_trajectory;
  std::vector<std::pair<int, double>> traj_contrib;
  AdaptiveSubdivideTrajectory(params.trajectory,
                              params.fingers,
                              settings.cost.d_linearity,
                              new_trajectory,
                              traj_contrib);
  long long n_trajectory = new_trajectory.size();
  std::vector<Fingers> new_fingers(n_trajectory);
  std::vector<Eigen::Affine3d> new_trans(n_trajectory);

#pragma omp parallel for
  for (size_t i = 0; i < new_trajectory.size(); i++) {
    new_trans[i] = robots::Forward(new_trajectory[i]);
    new_fingers[i] = TransformFingers(fingers, new_trans[i]);
  }

  // ========================
  // Gripper Energy
  // ========================
  double gripper_energy = 0;

  if (settings.cost.gripper_energy != 0.) {
    // std::vector<bool> traj_skip(n_trajectory - 1, false);
    std::vector<double> traj_devs(n_trajectory - 1);
    // std::vector<size_t> traj_subs(n_trajectory - 1, 0);
    double total_dev = 0;

    // #pragma omp parallel for
    for (long long i = 0; i < n_trajectory - 1; i++) {
      double max_deviation = 0;
      for (size_t j = 0; j < new_fingers[i].size(); j++) {
        double dev = (new_fingers[i + 1][j] - new_fingers[i][j])
                         .rowwise()
                         .squaredNorm()
                         .maxCoeff();
        max_deviation = std::max(max_deviation, dev);
      }

      max_deviation = sqrt(max_deviation);

      total_dev += max_deviation;
      traj_devs[i] = max_deviation;
    }
    std::vector<Pose> poses;

    double d_sub = (settings.cost.use_adaptive_subdivision)
                       ? settings.cost.d_subdivision
                       : total_dev / settings.cost.n_finger_steps;
    poses.reserve(total_dev / d_sub + 32);
    for (long long i = 0; i < n_trajectory - 1; i++) {
      size_t cur_sub = std::ceil(traj_devs[i] / d_sub);
      size_t iters = cur_sub;
      if (i == n_trajectory - 2) iters++;
      for (size_t j = 0; j < iters; j++) {
        double t = (double)j / cur_sub;
        Pose pose = new_trajectory[i] * (1. - t) + new_trajectory[i + 1] * t;
        poses.push_back(pose);
      }
    }
    long long n_poses = poses.size();

#pragma omp parallel
    {
      double t_max = 0;

#pragma omp for nowait
      for (long long j = 0; j < n_poses; j++) {
        auto f = TransformFingers(fingers, robots::Forward(poses[j]));
        t_max = std::max(t_max, ProcessFinger(f));
      }
#pragma omp critical
      gripper_energy = std::max(gripper_energy, t_max);
    }
  }

  // ========================
  // Trajectory Energy
  // ========================
  double trajectory_energy = 0;

  if (settings.cost.traj_energy != 0.) {
    std::vector<Eigen::Vector3d> d_fingers;
    size_t traj_subs = 0;
    for (size_t i = 0; i < fingers.size(); i++) {
      double total_norm = 0;
      for (Eigen::Index j = 1; j < fingers[i].rows(); j++) {
        double norm = (fingers[i].row(j) - fingers[i].row(j - 1)).norm();
        total_norm += norm;
      }
      double d_sub =
          settings.cost.use_adaptive_subdivision
              ? settings.cost.d_subdivision
              : total_norm * fingers.size() / settings.cost.n_trajectory_steps;
      for (Eigen::Index j = 1; j < fingers[i].rows(); j++) {
        double norm = (fingers[i].row(j) - fingers[i].row(j - 1)).norm();
        size_t subs = std::ceil(norm / d_sub);
        traj_subs += subs;
        for (size_t k = (j == 1) ? 0 : 1; k <= subs; k++) {
          double t = (double)k / subs;
          d_fingers.push_back(fingers[i].row(j - 1) * (1. - t) +
                              fingers[i].row(j) * t);
        }
      }
    }

#pragma omp parallel
    {
      double t_max = 0;
      _SegState state;

#pragma omp for nowait
      for (size_t j = 0; j < d_fingers.size(); j++) {
        Eigen::Vector3d p0 = new_trans[0] * d_fingers[j];
        state.is_first = true;
        double cur_cost = 0;
        for (size_t k = 1; k < new_trajectory.size(); k++) {
          Eigen::Vector3d p1 = new_trans[k] * d_fingers[j];
          cur_cost += MyCost(p0, p1, state, Eigen::RowVector3d(1, 0, 0.5));
          p0 = p1;
        }
        t_max = std::max(t_max, cur_cost);
      }

#pragma omp critical
      trajectory_energy = std::max(trajectory_energy, t_max);
    }
  }

  // ========================
  // Robot floor collision
  // ========================
  double robot_floor = 0;
  double max_penetration = 0;
  constexpr double robot_clearance = 0.05;
  for (const auto& trans : new_trans) {
    max_penetration =
        std::max(max_penetration,
                 std::max(0., robot_clearance - trans.translation()(1)));
  }
  robot_floor = max_penetration;

  // ========================
  // L2 regularization term
  // ========================
  double traj_reg = 0;
  for (size_t i = 1; i < params.trajectory.size() - 1; i++) {
    traj_reg += (params.trajectory[i] - init_params.trajectory[i])
                    .matrix()
                    .squaredNorm();
  }

  // ========================
  // Total Cost
  // ========================
  return settings.cost.gripper_energy * gripper_energy +
         settings.cost.traj_energy * trajectory_energy +
         settings.cost.robot_collision * robot_floor +
         settings.cost.regularization * traj_reg;
}

}  // namespace psg
