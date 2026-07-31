/*********************************************************************
 * Simplified collision model for the ninezzhou 9-DOF robot arm.
 *
 * Mirrors the Python robot_model / collision_checker for fair
 * comparison.  Uses joint limits + FK-based obstacle clearance.
 * The authoritative collision check is MoveIt2 + FCL.
 *********************************************************************/

#ifndef AEB_RRTSTAR_OMPL_NINEZZHOU_COLLISION_H_
#define AEB_RRTSTAR_OMPL_NINEZZHOU_COLLISION_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <array>
#include <vector>
#include <cmath>

namespace aeb_rrtstar_ompl
{

// ---------------------------------------------------------------------------
//  Joint limits (from ninezzhou.urdf)
// ---------------------------------------------------------------------------
constexpr unsigned int NINEZZHOU_DIM = 9;

struct JointLimit
{
    double lower;
    double upper;
};

constexpr std::array<JointLimit, NINEZZHOU_DIM> JOINT_LIMITS = {{
    {0.0, 0.585},          // J1  prismatic  (m)
    {-1.5708, 1.5708},     // J2  revolute   (rad)
    {-1.5708, 1.5708},     // J3  revolute   (rad)
    {-1.5708, 1.5708},     // J4  revolute   (rad)
    {-3.1416, 3.1416},     // J5  revolute   (rad) — full circle
    {-1.48353, 1.48353},   // J6  revolute   (rad)
    {-1.48353, 1.48353},   // J7  revolute   (rad)
    {-1.48353, 1.48353},   // J8  revolute   (rad)
    {-1.48353, 1.48353},   // J9  revolute   (rad)
}};

// ---------------------------------------------------------------------------
//  Obstacle types
// ---------------------------------------------------------------------------
struct BoxObstacle
{
    double cx, cy, cz;      // centre (Y-up base_link coords)
    double hx, hy, hz;      // half-extents
};

struct SphereObstacle
{
    double cx, cy, cz;
    double radius;
};

struct CylinderObstacle
{
    double cx, cy, cz;      // centre
    double radius;
    double half_height;     // half-height along Z (cylinder axis)
};

// Obstacles from config/obstacles.yaml, converted to half-extents.
// YAML convention: cylinder dimensions = [height, radius]
const std::vector<BoxObstacle> BOX_OBSTACLES = {
    {0.25, 0.243, 0.4, 0.04, 0.04, 0.08},    // obs_box1
    {-0.1, 0.15, 0.9, 0.05, 0.05, 0.05},      // obs_box2
};

const std::vector<SphereObstacle> SPHERE_OBSTACLES = {
    {-0.25, 0.343, 0.6, 0.05},                 // obs_sphere1
};

const std::vector<CylinderObstacle> CYLINDER_OBSTACLES = {
    {0.22, 0.30, 0.9, 0.03, 0.08},             // obs_cyl1
};

constexpr double LINK_CLEARANCE_M = 0.001;  // 1 mm

// ---------------------------------------------------------------------------
//  Simplified forward kinematics
// ---------------------------------------------------------------------------

/** Return Cartesian positions of joint origins + tool0 (11 points). */
inline std::vector<std::array<double, 3>> forwardKinematics(
    const double *joints)
{
    std::vector<std::array<double, 3>> pts;
    pts.reserve(11);

    double p[3] = {0.0, 0.0, joints[0]};  // J1 prismatic Z
    pts.push_back({p[0], p[1], p[2]});

    // Rotation matrix (starts as identity)
    double R[9] = {1,0,0, 0,1,0, 0,0,1};

    // Link offsets from URDF (parent-frame translations)
    const double offsets[9][3] = {
        {0.0, 0.343, 0.0},     // J2
        {0.225, 0.0, 0.0},     // J3
        {0.225, 0.0, 0.0},     // J4
        {0.0, -0.343, 0.0},    // J5
        {0.0, 0.0, 0.0},       // J6
        {0.135, 0.0, 0.0},     // J7
        {0.11, 0.0, 0.0},      // J8
        {0.114, 0.0, 0.0},     // J9
        {0.235, 0.0, 0.0},     // tool0
    };

    for (int i = 1; i < 9; ++i)
    {
        // Apply offset
        p[0] += R[0]*offsets[i-1][0] + R[1]*offsets[i-1][1] + R[2]*offsets[i-1][2];
        p[1] += R[3]*offsets[i-1][0] + R[4]*offsets[i-1][1] + R[5]*offsets[i-1][2];
        p[2] += R[6]*offsets[i-1][0] + R[7]*offsets[i-1][1] + R[8]*offsets[i-1][2];

        // Rotate by joint angle about Z
        double theta = joints[i];
        double c = std::cos(theta), s = std::sin(theta);
        double Rz[9] = {c, -s, 0, s, c, 0, 0, 0, 1};
        // R = R * Rz
        double tmp[9];
        for (int r = 0; r < 3; ++r)
            for (int col = 0; col < 3; ++col)
                tmp[r*3+col] = R[r*3+0]*Rz[0*3+col]
                             + R[r*3+1]*Rz[1*3+col]
                             + R[r*3+2]*Rz[2*3+col];
        for (int k = 0; k < 9; ++k) R[k] = tmp[k];

        pts.push_back({p[0], p[1], p[2]});
    }

    // tool0
    p[0] += R[0]*offsets[8][0] + R[1]*offsets[8][1] + R[2]*offsets[8][2];
    p[1] += R[3]*offsets[8][0] + R[4]*offsets[8][1] + R[5]*offsets[8][2];
    p[2] += R[6]*offsets[8][0] + R[7]*offsets[8][1] + R[8]*offsets[8][2];
    pts.push_back({p[0], p[1], p[2]});

    return pts;
}

// ---------------------------------------------------------------------------
//  Obstacle distance queries
// ---------------------------------------------------------------------------
inline double distBox(const std::array<double,3> &pt,
                      const BoxObstacle &box)
{
    double dx = std::max(0.0, std::abs(pt[0] - box.cx) - box.hx);
    double dy = std::max(0.0, std::abs(pt[1] - box.cy) - box.hy);
    double dz = std::max(0.0, std::abs(pt[2] - box.cz) - box.hz);
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

inline double distSphere(const std::array<double,3> &pt,
                         const SphereObstacle &sph)
{
    double dx = pt[0] - sph.cx;
    double dy = pt[1] - sph.cy;
    double dz = pt[2] - sph.cz;
    return std::max(0.0, std::sqrt(dx*dx + dy*dy + dz*dz) - sph.radius);
}

inline double distCylinder(const std::array<double,3> &pt,
                           const CylinderObstacle &cyl)
{
    double dx = pt[0] - cyl.cx;
    double dy = pt[1] - cyl.cy;
    double dz = pt[2] - cyl.cz;
    double rdist = std::sqrt(dx*dx + dy*dy);
    double adist = std::abs(dz);
    double d_radial = std::max(0.0, rdist - cyl.radius);
    double d_axial  = std::max(0.0, adist - cyl.half_height);
    if (rdist <= cyl.radius) return d_axial;
    if (adist <= cyl.half_height) return d_radial;
    return std::sqrt(d_radial*d_radial + d_axial*d_axial);
}

inline double minObstacleClearance(const std::array<double,3> &pt)
{
    double best = 1e9;
    for (const auto &b : BOX_OBSTACLES)
        best = std::min(best, distBox(pt, b));
    for (const auto &s : SPHERE_OBSTACLES)
        best = std::min(best, distSphere(pt, s));
    for (const auto &c : CYLINDER_OBSTACLES)
        best = std::min(best, distCylinder(pt, c));
    return best;
}

// ---------------------------------------------------------------------------
//  Configuration validity
// ---------------------------------------------------------------------------
inline bool isConfigValid(const double *joints)
{
    // Joint limits
    for (unsigned int i = 0; i < NINEZZHOU_DIM; ++i)
    {
        if (joints[i] < JOINT_LIMITS[i].lower - 1e-10 ||
            joints[i] > JOINT_LIMITS[i].upper + 1e-10)
            return false;
    }

    // FK obstacle clearance
    auto pts = forwardKinematics(joints);
    for (const auto &pt : pts)
    {
        if (minObstacleClearance(pt) < LINK_CLEARANCE_M)
            return false;
    }
    return true;
}

inline bool isMotionValid(const double *from, const double *to, int steps = 16)
{
    if (!isConfigValid(to)) return false;
    int n = std::max(steps, 2);
    for (int k = 1; k < n; ++k)
    {
        double t = static_cast<double>(k) / n;
        double interp[NINEZZHOU_DIM];
        for (unsigned int i = 0; i < NINEZZHOU_DIM; ++i)
            interp[i] = from[i] + t * (to[i] - from[i]);
        if (!isConfigValid(interp)) return false;
    }
    return true;
}

// ======================================================================
//  OMPL StateValidityChecker
// ======================================================================
class NinezzhouStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    NinezzhouStateValidityChecker(const ompl::base::SpaceInformationPtr &si)
      : ompl::base::StateValidityChecker(si) {}

    bool isValid(const ompl::base::State *state) const override
    {
        const auto *rv =
            state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        return aeb_rrtstar_ompl::isConfigValid(rv);
    }
};

// ======================================================================
//  OMPL MotionValidator
// ======================================================================
class NinezzhouMotionValidator : public ompl::base::MotionValidator
{
public:
    NinezzhouMotionValidator(const ompl::base::SpaceInformationPtr &si)
      : ompl::base::MotionValidator(si) {}

    bool checkMotion(const ompl::base::State *s1,
                     const ompl::base::State *s2) const override
    {
        const auto *a =
            s1->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        const auto *b =
            s2->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        return isMotionValid(a, b);
    }

    bool checkMotion(const ompl::base::State *s1,
                     const ompl::base::State *s2,
                     std::pair<ompl::base::State *, double> &lastValid) const override
    {
        (void)lastValid;
        return checkMotion(s1, s2);
    }
};

}  // namespace aeb_rrtstar_ompl

#endif  // AEB_RRTSTAR_OMPL_NINEZZHOU_COLLISION_H_
