// @lat: [[process/test-strategy#skill round-trip]]
//
// mill_open_pocket — a notch carved from the side of the entry face.
//
// Tests:
//   1. Apply produces an open-sided notch (volume removed ≈ length × width × depth).
//   2. DFM rejects undersized corner R (DFM-004).
//   3. DFM rejects width that can't accommodate 2 corner fillets.
//   4. Recognize finds the 2-corner pattern (vs 4 for closed pockets).
//   5. Edge case: sharp-corner notch (corner_r=0) applies cleanly.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mill_open_pocket.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}
}  // namespace

// ─── 1. Basic synthesis works ─────────────────────────────────────────────
TEST(SkillMillOpenPocket, ApplyCarvesNotchFromEdge)
{
    // 60 × 40 × 10 stock.  Notch on the +X side, 10 mm into the workpiece,
    // 12 mm wide along Y, 3 mm deep, centred at Y=20.
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::mill_open_pocket::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 60.0;          // open edge sits at the +X side
    in.position_y_mm  = 20.0;          // centred Y
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.open_direction = gp_Dir(1, 0, 0);
    in.length_mm      = 10.0;
    in.width_mm       = 12.0;
    in.depth_mm       = 3.0;
    in.corner_r_mm    = 1.0;

    auto out = skill::mill_open_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approx = in.length_mm * in.width_mm * in.depth_mm;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    // Corners are rounded → slightly less than box approximation, but the
    // open side ADDS some volume removal (no closing wall), so net is close
    // to the box approximation within ~ 10 %.
    EXPECT_NEAR(removed, approx, approx * 0.15);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("mill_open_pocket"));
    EXPECT_EQ(out.signature.pattern["planar_wall_count"].get<int>(), 3);
    EXPECT_EQ(out.signature.pattern["corner_cylinder_count"].get<int>(), 2);
}

// ─── 2. DFM rejects sub-min corner R ─────────────────────────────────────
TEST(SkillMillOpenPocket, ValidateRejectsSmallCornerR)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::mill_open_pocket::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 60.0;
    in.position_y_mm  = 20.0;
    in.open_direction = gp_Dir(1, 0, 0);
    in.length_mm      = 10.0;
    in.width_mm       = 12.0;
    in.depth_mm       = 3.0;
    in.corner_r_mm    = 0.1;            // < 0.2 mm

    auto r = skill::mill_open_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-004") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::mill_open_pocket::apply(*stock, in), skill::SkillError);
}

// ─── 3. Width must accommodate corner fillets ────────────────────────────
TEST(SkillMillOpenPocket, ValidateRejectsCollapsedWidth)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::mill_open_pocket::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 60.0;
    in.position_y_mm  = 20.0;
    in.open_direction = gp_Dir(1, 0, 0);
    in.length_mm      = 10.0;
    in.width_mm       = 4.0;             // too narrow for 2 × 3 mm radii
    in.depth_mm       = 3.0;
    in.corner_r_mm    = 3.0;             // 2·r = 6 mm > width 4 mm

    auto r = skill::mill_open_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::mill_open_pocket::apply(*stock, in), skill::SkillError);
}

// ─── 4. Recognize finds the 2-corner pattern ─────────────────────────────
TEST(SkillMillOpenPocket, RecognizeFindsOpenNotch)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::mill_open_pocket::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 60.0;
    in.position_y_mm  = 20.0;
    in.open_direction = gp_Dir(1, 0, 0);
    in.length_mm      = 10.0;
    in.width_mm       = 12.0;
    in.depth_mm       = 3.0;
    in.corner_r_mm    = 1.0;

    auto out = skill::mill_open_pocket::apply(*stock, in);
    auto cands = skill::mill_open_pocket::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty()) << "expected at least one open-pocket candidate";

    // Should match by corner_r and depth.
    bool found = false;
    for (const auto& c : cands) {
        const double r     = c.recovered_params["corner_r_mm"].get<double>();
        const double depth = c.recovered_params["depth_mm"].get<double>();
        const double width = c.recovered_params["width_mm"].get<double>();
        if (std::abs(r - 1.0) < 0.1 &&
            std::abs(depth - 3.0) < 0.1 &&
            std::abs(width - 12.0) < 1.0)
        {
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found) << "no candidate matched the synthesized open-pocket params";
}

// ─── 5. Sharp-corner notch (corner_r=0) applies ──────────────────────────
TEST(SkillMillOpenPocket, SharpCornerZeroRadiusApplies)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::mill_open_pocket::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 0.0;             // open on -X side
    in.position_y_mm  = 20.0;
    in.open_direction = gp_Dir(-1, 0, 0);
    in.length_mm      = 8.0;
    in.width_mm       = 10.0;
    in.depth_mm       = 2.5;
    in.corner_r_mm    = 0.0;             // sharp corners — documented choice

    auto r = skill::mill_open_pocket::validate(*stock, in);
    EXPECT_TRUE(r.passed);

    auto out = skill::mill_open_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approx = in.length_mm * in.width_mm * in.depth_mm;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.10);

    // Sharp corners → corner_cylinder_count == 0
    EXPECT_EQ(out.signature.pattern["corner_cylinder_count"].get<int>(), 0);
}
