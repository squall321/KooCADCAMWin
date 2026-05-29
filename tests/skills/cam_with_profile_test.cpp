// @lat: [[process/test-strategy#skill round-trip]]
//
// cam_with_profile: real-geometry compound feature tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cam_with_profile.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

// Build a simple eccentric cam profile: base radius 15, with a smooth
// bump up to 20 over θ ∈ [π/2, π].  16 waypoints.
std::vector<skill::cam_with_profile::Waypoint> makeEccentricCam()
{
    std::vector<skill::cam_with_profile::Waypoint> p;
    const int N = 16;
    for (int i = 0; i < N; ++i) {
        skill::cam_with_profile::Waypoint w;
        w.theta_rad = 2.0 * M_PI * i / N;
        const double phase = (w.theta_rad - M_PI / 2.0);
        // Smooth bump near θ ≈ 3π/4: r = 15 + 5·max(0, cos(2(θ-3π/4)))
        const double bump = std::max(0.0,
            std::cos(2.0 * (w.theta_rad - 3.0 * M_PI / 4.0)));
        w.r_mm = 15.0 + 5.0 * bump;
        (void)phase;
        p.push_back(w);
    }
    return p;
}

// Polygon area via shoelace, given vertices.
double polygonArea(const std::vector<skill::cam_with_profile::Waypoint>& p,
                   double cx, double cy)
{
    double a = 0.0;
    const int n = static_cast<int>(p.size());
    for (int i = 0; i < n; ++i) {
        const auto& A = p[i];
        const auto& B = p[(i + 1) % n];
        const double ax = cx + A.r_mm * std::cos(A.theta_rad);
        const double ay = cy + A.r_mm * std::sin(A.theta_rad);
        const double bx = cx + B.r_mm * std::cos(B.theta_rad);
        const double by = cy + B.r_mm * std::sin(B.theta_rad);
        a += ax * by - bx * ay;
    }
    return std::abs(a) * 0.5;
}

}  // namespace

// Test 1: real compound geometry — cam disc (prism over profile) - shaft bore.
TEST(SkillCamWithProfile, ApplyProducesRealCompoundGeometry)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 4.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::cam_with_profile::Input in;
    in.profile      = makeEccentricCam();
    in.thickness_mm = 10.0;
    in.shaft_dia_mm = 8.0;
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 30.0;
    in.center_z_mm  = 4.0;

    auto out = skill::cam_with_profile::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double areaCam   = polygonArea(in.profile, in.center_x_mm, in.center_y_mm);
    const double vCamAdd   = areaCam * in.thickness_mm;
    const double rS        = in.shaft_dia_mm / 2.0;
    const double vShaft    = M_PI * rS * rS * in.thickness_mm;
    const double expected  = vCamAdd - vShaft;

    const double actual = volumeOf(out.workpiece->shape()) - volumeOf(stock->shape());
    EXPECT_NEAR(actual, expected, std::abs(expected) * 0.20)
        << "expected " << expected << ", actual " << actual;

    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// Test 2: DFM rejects too-few waypoints.
TEST(SkillCamWithProfile, ValidateRejectsTooFewPoints)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 4.0);

    skill::cam_with_profile::Input in;
    // Just 4 waypoints — below DFM threshold of 8.
    for (int i = 0; i < 4; ++i) {
        skill::cam_with_profile::Waypoint w;
        w.theta_rad = 2.0 * M_PI * i / 4.0;
        w.r_mm = 15.0;
        in.profile.push_back(w);
    }
    in.thickness_mm = 10.0;
    in.shaft_dia_mm = 8.0;

    auto r = skill::cam_with_profile::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CAM-POINTS") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::cam_with_profile::apply(*stock, in),
                 skill::SkillError);
}

// Test 3: signature compound + round-trip params.
TEST(SkillCamWithProfile, SignatureCompound)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 4.0);

    skill::cam_with_profile::Input in;
    in.profile      = makeEccentricCam();
    in.thickness_mm = 10.0;
    in.shaft_dia_mm = 8.0;
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 30.0;
    in.center_z_mm  = 4.0;

    auto out = skill::cam_with_profile::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("cam_with_profile"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_GE(out.signature.pattern.value("subfeature_count", 0), 2);

    EXPECT_EQ(out.signature.params["waypoint_count"].get<int>(),
              static_cast<int>(in.profile.size()));
    EXPECT_NEAR(out.signature.params["shaft_dia_mm"].get<double>(),
                in.shaft_dia_mm, 1e-9);
}

// Test 4: recognize finds the cam.
TEST(SkillCamWithProfile, RecognizeFindsCam)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 4.0);

    skill::cam_with_profile::Input in;
    in.profile      = makeEccentricCam();
    in.thickness_mm = 10.0;
    in.shaft_dia_mm = 8.0;
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 30.0;
    in.center_z_mm  = 4.0;

    auto out = skill::cam_with_profile::apply(*stock, in);
    auto cands = skill::cam_with_profile::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands.front().confidence, 0.5);
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
}

// Test 5: profile bbox matches waypoint extents.
TEST(SkillCamWithProfile, ProfileExtentsMatchWaypoints)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 4.0);

    skill::cam_with_profile::Input in;
    in.profile      = makeEccentricCam();
    in.thickness_mm = 10.0;
    in.shaft_dia_mm = 8.0;
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 30.0;
    in.center_z_mm  = 4.0;

    auto out = skill::cam_with_profile::apply(*stock, in);

    // Find the maximum waypoint radius.
    double maxR = 0.0;
    for (const auto& w : in.profile) maxR = std::max(maxR, w.r_mm);

    // The shape bounding box X extent should be at least 2·maxR (the
    // cam disc width).
    double xMin, yMin, zMin, xMax, yMax, zMax;
    out.workpiece->boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    EXPECT_GE(xMax - xMin, 2.0 * maxR - 1.0)
        << "cam X-extent should match profile diameter";

    // Pattern reports max_radius_mm correctly.
    EXPECT_NEAR(out.signature.pattern.value("max_radius_mm", 0.0), maxR, 1e-9);
}
