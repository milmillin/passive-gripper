#pragma once

#define DECLARE_GETTER(x, y) \
  inline constexpr const decltype(y)& x() const { return y; }

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <map>
#include <vector>

namespace psg {

constexpr double kPi = EIGEN_PI;
constexpr double kTwoPi = kPi * 2.;
constexpr double kHalfPi = kPi / 2.;
const double kDegToRad = kPi / 180.;
const double kRadToDeg = 180. / kPi;

constexpr size_t kNumDOFs = 6;
const double kRobotA[] = {0, -0.425, -0.39225, 0, 0, 0};
const double kRobotD[] = {0.089159, 0, 0, 0.10915, 0.09465, 0.0823};
const double kRobotAlpha[] = {kHalfPi, 0, 0, kHalfPi, -kHalfPi, 0};

const double kArmRadius = 0.0465;
const Eigen::Affine3d kLocalTrans[6] = {
    Eigen::Translation3d(-kArmRadius, -0.0892, -kArmRadius) *
        Eigen::Scaling(2 * kArmRadius, kArmRadius + 0.0892, 2 * kArmRadius),
    Eigen::Translation3d(-kArmRadius, -kArmRadius, kArmRadius) *
        Eigen::Scaling(2 * kArmRadius + 0.425, 2 * kArmRadius, 2 * kArmRadius),
    Eigen::Translation3d(-kArmRadius, -kArmRadius, 0.02 - kArmRadius) *
        Eigen::Scaling(2 * kArmRadius + 0.39225,
                       2 * kArmRadius,
                       2 * kArmRadius),
    Eigen::Translation3d(-kArmRadius, -kArmRadius, -kArmRadius) *
        Eigen::Scaling(2 * kArmRadius, 2 * kArmRadius, 0.09465),
    Eigen::Translation3d(-kArmRadius, -kArmRadius, -kArmRadius) *
        Eigen::Scaling(2 * kArmRadius,
                       2 * kArmRadius,
                       kArmRadius + 0.0825 - 0.01),
    Eigen::Translation3d(-kArmRadius, -kArmRadius, -0.01) *
        Eigen::Scaling(2 * kArmRadius, 2 * kArmRadius, 0.01)};

// https://roboticscasual.com/files/ur5_rviz.txt
const Eigen::Affine3d kUrdfTrans[6] = {
    (Eigen::Affine3d)Eigen::Translation3d(0, 0, 0.089159),
    Eigen::Translation3d(0, 0.13585, 0) *
        Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitY()),
    (Eigen::Affine3d)Eigen::Translation3d(0, -0.1197, 0.425),
    Eigen::Translation3d(0, 0, 0.39225) *
        Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitY()),
    (Eigen::Affine3d)Eigen::Translation3d(0, 0.093, 0),
    (Eigen::Affine3d)Eigen::Translation3d(0, 0, 0.09465)};

const Eigen::Vector3d kUrdfAxis[6] = {
    Eigen::Vector3d::UnitZ(),
    Eigen::Vector3d::UnitY(),
    Eigen::Vector3d::UnitY(),
    Eigen::Vector3d::UnitY(),
    Eigen::Vector3d::UnitZ(),
    Eigen::Vector3d::UnitY(),
};

typedef Eigen::Array<double, kNumDOFs, 1> Pose;
typedef std::vector<Pose> Trajectory;
typedef std::vector<Eigen::MatrixXd> Fingers;

// Jacobian 3x6 matrix [dpos/dtheta_0 | ... | dpos/dtheta_5]
typedef Eigen::Matrix<double, 3, kNumDOFs> Jacobian;

// Returns Jacobian (3x6 matrix) given pos in effector space
typedef std::function<Jacobian(const Eigen::Vector3d&)> JacobianFunc;

const Pose kInitPose =
    (Pose() << -kHalfPi, -2., -2., 4., -kHalfPi, 0.).finished();

// Quality Metric
// small float to make quadratic program positive semidefinite
static constexpr double kWrenchReg = 1e-10;
// zero threshold
static constexpr double kWrenchNormThresh = 1e-5;

constexpr double kExpandMesh = 0.002; // 2mm

// GC Generations
constexpr size_t kNSeeds = 1000;
constexpr size_t kNCandidates = 3000;

const std::map<std::string, std::string> kTopyConfig = {{"PROB_TYPE", "comp"},
                                                        {"ETA", "0.4"},
                                                        {"DOF_PN", "3"},
                                                        {"FILT_RAD", "1.8"},
                                                        {"ELEM_K", "H8"},
                                                        {"NUM_ITER", "50"},
                                                        {"P_FAC", "1"},
                                                        {"P_HOLD", "15"},
                                                        {"P_INCR", "0.2"},
                                                        {"P_CON", "1"},
                                                        {"P_MAX", "3"},
                                                        {"Q_FAC", "1"},
                                                        {"Q_HOLD", "15"},
                                                        {"Q_INCR", "0.05"},
                                                        {"Q_CON", "1"},
                                                        {"Q_MAX", "5"}};

const char* const kBoolStr[2] = {"False", "True"};

namespace labels {

const char* const kAlgorithms[] = {
    "NLOPT_GN_DIRECT",
    "NLOPT_GN_DIRECT_L",
    "NLOPT_GN_DIRECT_L_RAND",
    "NLOPT_GN_DIRECT_NOSCAL",
    "NLOPT_GN_DIRECT_L_NOSCAL",
    "NLOPT_GN_DIRECT_L_RAND_NOSCAL",

    "NLOPT_GN_ORIG_DIRECT",
    "NLOPT_GN_ORIG_DIRECT_L",

    "NLOPT_GD_STOGO",
    "NLOPT_GD_STOGO_RAND",

    "NLOPT_LD_LBFGS_NOCEDAL",

    "NLOPT_LD_LBFGS",

    "NLOPT_LN_PRAXIS",

    "NLOPT_LD_VAR1",
    "NLOPT_LD_VAR2",

    "NLOPT_LD_TNEWTON",
    "NLOPT_LD_TNEWTON_RESTART",
    "NLOPT_LD_TNEWTON_PRECOND",
    "NLOPT_LD_TNEWTON_PRECOND_RESTART",

    "NLOPT_GN_CRS2_LM",

    "NLOPT_GN_MLSL",
    "NLOPT_GD_MLSL",
    "NLOPT_GN_MLSL_LDS",
    "NLOPT_GD_MLSL_LDS",

    "NLOPT_LD_MMA",

    "NLOPT_LN_COBYLA",

    "NLOPT_LN_NEWUOA",
    "NLOPT_LN_NEWUOA_BOUND",

    "NLOPT_LN_NELDERMEAD",
    "NLOPT_LN_SBPLX",

    "NLOPT_LN_AUGLAG",
    "NLOPT_LD_AUGLAG",
    "NLOPT_LN_AUGLAG_EQ",
    "NLOPT_LD_AUGLAG_EQ",

    "NLOPT_LN_BOBYQA",

    "NLOPT_GN_ISRES",

    /* new variants that require local_optimizer to be set",
       not with older constants for backwards compatibility */
    "NLOPT_AUGLAG",
    "NLOPT_AUGLAG_EQ",
    "NLOPT_G_MLSL",
    "NLOPT_G_MLSL_LDS",

    "NLOPT_LD_SLSQP",

    "NLOPT_LD_CCSAQ",

    "NLOPT_GN_ESCH",

    "NLOPT_GN_AGS"};
}

enum class CostFunctionEnum : int { kGradientBased = 0, kSP = 1 };

namespace colors {
const Eigen::RowVector3d kPurple = Eigen::RowVector3d(219, 76, 178) / 255;
const Eigen::RowVector3d kOrange = Eigen::RowVector3d(239, 126, 50) / 255;
const Eigen::RowVector3d kRed = Eigen::RowVector3d(192, 35, 35) / 255;
const Eigen::RowVector3d kGreen = Eigen::RowVector3d(35, 192, 35) / 255;
const Eigen::RowVector3d kBrown = Eigen::RowVector3d(130, 4, 1) / 255;
const Eigen::RowVector3d kDarkBlue = Eigen::RowVector3d(20, 36, 89) / 255;
const Eigen::RowVector3d kBlue = Eigen::RowVector3d(0, 0, 255) / 255;
const Eigen::RowVector3d kGold = Eigen::RowVector3d(252, 181, 9) / 255;
const Eigen::RowVector3d kWhite = Eigen::RowVector3d(255, 255, 255) / 255;
}  // namespace colors
}  // namespace psg
