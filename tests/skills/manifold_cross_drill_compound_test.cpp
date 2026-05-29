// @lat: [[process/test-strategy#compound macro]]
//
// manifold_cross_drill_compound — 2 perpendicular through-drills + 2
// plug counterbores on the active end faces.
//
// Test cases:
//   1. apply produces a valid shape; volume removed AND face-count change.
//   2. validate rejects non-perpendicular axes / unknown plug_M.
//   3. signature records is_compound + subfeature_count == 4.
//   4. recognize replays metadata at confidence 1.0.
//   5. Pattern carries derived plug_seat_dia_mm (from plug_M lookup).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/manifold_cross_drill_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. apply produces valid shape with multiple sub-features ──────────────
TEST(SkillManifoldCrossDrillCompound, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    // 60x60x60 block, plenty of room for 6 mm cross drills and M8 plugs.
    auto stock = skill::createCuboidStock(60.0, 60.0, 60.0);
    const double v0   = volumeOf(stock->shape());
    const int    f0   = stock->faceCount();

    skill::manifold_cross_drill_compound::Input in;
    in.block_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.drill1_axis         = gp_Dir(1, 0, 0);
    in.drill2_axis         = gp_Dir(0, 1, 0);
    in.drill1_origin_x_mm  = 30.0;
    in.drill1_origin_y_mm  = 30.0;
    in.drill1_origin_z_mm  = 30.0;
    in.drill_dia_mm        = 6.0;
    in.plug_M              = "M8";
    in.plug_seat_depth_mm  = 4.0;

    auto out = skill::manifold_cross_drill_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 0.0);                  // material was removed
    // Face count must change — two through-passages + 2 seats add many faces.
    EXPECT_NE(out.workpiece->faceCount(), f0);
}

// ─── 2. validate rejects non-perpendicular axes & unknown plug ─────────────
TEST(SkillManifoldCrossDrillCompound, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 60.0);

    skill::manifold_cross_drill_compound::Input bad;
    bad.block_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.drill1_axis         = gp_Dir(1, 0, 0);
    bad.drill2_axis         = gp_Dir(1, 0, 0);   // NOT perpendicular
    bad.drill1_origin_x_mm  = 30.0;
    bad.drill1_origin_y_mm  = 30.0;
    bad.drill1_origin_z_mm  = 30.0;
    bad.drill_dia_mm        = 6.0;
    bad.plug_M              = "M8";
    bad.plug_seat_depth_mm  = 4.0;
    auto r = skill::manifold_cross_drill_compound::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::manifold_cross_drill_compound::apply(*stock, bad),
                 skill::SkillError);

    // Also reject unknown plug.
    skill::manifold_cross_drill_compound::Input bad2 = bad;
    bad2.drill2_axis = gp_Dir(0, 1, 0);   // fix perpendicularity
    bad2.plug_M      = "M99";              // unknown
    auto r2 = skill::manifold_cross_drill_compound::validate(*stock, bad2);
    EXPECT_FALSE(r2.passed);
    bool found = false;
    for (const auto& f : r2.findings)
        if (f.code == "DFM-MANIFOLD-PLUG") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Signature records compound kind ────────────────────────────────────
TEST(SkillManifoldCrossDrillCompound, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 60.0);

    skill::manifold_cross_drill_compound::Input in;
    in.block_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.drill1_axis        = gp_Dir(1, 0, 0);
    in.drill2_axis        = gp_Dir(0, 1, 0);
    in.drill1_origin_x_mm = 30.0;
    in.drill1_origin_y_mm = 30.0;
    in.drill1_origin_z_mm = 30.0;
    in.drill_dia_mm       = 6.0;
    in.plug_M             = "M8";
    in.plug_seat_depth_mm = 4.0;

    auto out = skill::manifold_cross_drill_compound::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id,
              std::string("manifold_cross_drill_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
}

// ─── 4. Recognize replays metadata at confidence 1.0 ───────────────────────
TEST(SkillManifoldCrossDrillCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 60.0);

    skill::manifold_cross_drill_compound::Input in;
    in.block_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.drill1_axis        = gp_Dir(1, 0, 0);
    in.drill2_axis        = gp_Dir(0, 1, 0);
    in.drill1_origin_x_mm = 30.0;
    in.drill1_origin_y_mm = 30.0;
    in.drill1_origin_z_mm = 30.0;
    in.drill_dia_mm       = 6.0;
    in.plug_M             = "M8";
    in.plug_seat_depth_mm = 4.0;

    auto out = skill::manifold_cross_drill_compound::apply(*stock, in);

    auto cands = skill::manifold_cross_drill_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_EQ(cands.front().recovered_params["plug_M"].get<std::string>(),
              std::string("M8"));
}

// ─── 5. Derived plug_seat_dia_mm matches the standard table ───────────────
TEST(SkillManifoldCrossDrillCompound, PatternCarriesDerivedSeatDia)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 60.0);

    skill::manifold_cross_drill_compound::Input in;
    in.block_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.drill1_axis        = gp_Dir(1, 0, 0);
    in.drill2_axis        = gp_Dir(0, 1, 0);
    in.drill1_origin_x_mm = 30.0;
    in.drill1_origin_y_mm = 30.0;
    in.drill1_origin_z_mm = 30.0;
    in.drill_dia_mm       = 6.0;
    in.plug_M             = "M8";              // → seat_dia = 10.5 mm
    in.plug_seat_depth_mm = 4.0;

    auto out = skill::manifold_cross_drill_compound::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern.at("plug_seat_dia_mm").get<double>(),
                10.5, 1e-6);
    // Sanity: seat dia STRICTLY > drill dia (otherwise plug cannot seat).
    EXPECT_GT(out.signature.pattern.at("plug_seat_dia_mm").get<double>(),
              in.drill_dia_mm);
}
