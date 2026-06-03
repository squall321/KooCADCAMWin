// @lat: [[process/test-strategy#skill round-trip]]
//
// structural_member_along_path — sweep a parametric cross-section profile
// along a 3D path and FUSE onto the workpiece.
//
// Tests (5):
//   1. Apply: square_tube produces real added volume ≈ area × path within ±20 %.
//   2. DFM rejects profile_dim ≤ 2 × wall_thick.
//   3. DFM rejects path length ≤ profile dim.
//   4. Signature pattern carries is_compound + profile_kind + path_length.
//   5. Recognize replays the feature from history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/structural_member_along_path.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <array>
#include <cmath>
#include <vector>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply: square_tube real volume change ─────────────────────────────
TEST(SkillStructuralMemberAlongPath, ApplySquareTubeAddsRealVolume)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);
    const double v0 = volumeOf(stock->shape());

    skill::structural_member_along_path::Input in;
    in.profile_kind   = "square_tube";
    in.profile_dim_mm = 50.0;
    in.wall_thick_mm  = 4.0;
    // 500-mm straight path along +X starting OUTSIDE the stock.
    in.path_polyline_xyz = {
        std::array<double, 3>{ 200.0, 0.0, 0.0 },
        std::array<double, 3>{ 700.0, 0.0, 0.0 },
    };

    auto out = skill::structural_member_along_path::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // section area = 50² − 42² = 2500 − 1764 = 736 mm² ; × 500 mm = 368 000 mm³.
    const double area = 50.0 * 50.0 - 42.0 * 42.0;
    const double expected = area * 500.0;
    EXPECT_NEAR(v1 - v0, expected, expected * 0.20);
}

// ─── 2. DFM rejects too-thick wall (eats through profile) ─────────────────
TEST(SkillStructuralMemberAlongPath, ValidateRejectsWallTooThick)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::structural_member_along_path::Input in;
    in.profile_kind   = "square_tube";
    in.profile_dim_mm = 10.0;
    in.wall_thick_mm  = 6.0;   // 2 × 6 = 12 > 10 → fails DFM-WALL
    in.path_polyline_xyz = {
        std::array<double, 3>{ 100.0, 0.0, 0.0 },
        std::array<double, 3>{ 200.0, 0.0, 0.0 },
    };

    auto r = skill::structural_member_along_path::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::structural_member_along_path::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects too-short path ────────────────────────────────────────
TEST(SkillStructuralMemberAlongPath, ValidateRejectsShortPath)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::structural_member_along_path::Input in;
    in.profile_kind   = "square_solid";
    in.profile_dim_mm = 50.0;
    in.wall_thick_mm  = 0.0;
    in.path_polyline_xyz = {
        std::array<double, 3>{ 0.0,  0.0, 0.0 },
        std::array<double, 3>{ 30.0, 0.0, 0.0 },   // path 30 < dim 50
    };

    auto r = skill::structural_member_along_path::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature pattern ─────────────────────────────────────────────────
TEST(SkillStructuralMemberAlongPath, SignatureCompound)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::structural_member_along_path::Input in;
    in.profile_kind   = "round_tube";
    in.profile_dim_mm = 40.0;
    in.wall_thick_mm  = 3.0;
    in.path_polyline_xyz = {
        std::array<double, 3>{ 200.0, 0.0, 0.0 },
        std::array<double, 3>{ 500.0, 0.0, 0.0 },
    };

    auto out = skill::structural_member_along_path::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id,
              std::string("structural_member_along_path"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("profile_kind", std::string{}),
              std::string("round_tube"));
    EXPECT_NEAR(out.signature.pattern.value("path_length", 0.0), 300.0, 1e-6);
    EXPECT_GT(out.signature.pattern.value("derived_volume_mm3", 0.0), 0.0);
}

// ─── 5. Recognize replays history ─────────────────────────────────────────
TEST(SkillStructuralMemberAlongPath, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::structural_member_along_path::Input in;
    in.profile_kind   = "square_solid";
    in.profile_dim_mm = 30.0;
    in.wall_thick_mm  = 0.0;
    in.path_polyline_xyz = {
        std::array<double, 3>{ 200.0, 0.0, 0.0 },
        std::array<double, 3>{ 400.0, 0.0, 0.0 },
    };

    auto out   = skill::structural_member_along_path::apply(*stock, in);
    auto cands = skill::structural_member_along_path::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id,
              std::string("structural_member_along_path"));
    EXPECT_GT(cands.front().confidence, 0.4);
}
