// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/spot_face.hpp"

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

// ── 1. Apply: shallow circular seat created ──────────────────────────────
TEST(SkillSpotFace, ApplyCreatesShallowSeat)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::spot_face::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.dia_mm       = 12.0;
    in.depth_mm     = 1.0;

    auto out = skill::spot_face::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expected = M_PI * 6.0 * 6.0 * 1.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.05);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("spot_face"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("spot_face"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("spot_face_cutter"));
}

// ── 2. DFM-002: dia < 3 mm rejected ──────────────────────────────────────
TEST(SkillSpotFace, ValidateRejectsSmallDia)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::spot_face::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0; in.position_y_mm = 25.0;
    in.dia_mm = 2.5; in.depth_mm = 1.0;

    auto r = skill::spot_face::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::spot_face::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-SPOTFACE-DEPTH: depth > 5 mm rejected ─────────────────────────
TEST(SkillSpotFace, ValidateRejectsExcessiveDepth)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::spot_face::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0; in.position_y_mm = 30.0;
    in.dia_mm = 12.0; in.depth_mm = 6.0;     // > 5 mm → not a spot-face

    auto r = skill::spot_face::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPOTFACE-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. DFM-SPOTFACE-DEPTH: depth < 0.3 mm rejected ───────────────────────
TEST(SkillSpotFace, ValidateRejectsShallowSeat)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::spot_face::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0; in.position_y_mm = 25.0;
    in.dia_mm = 10.0; in.depth_mm = 0.15;   // too shallow

    auto r = skill::spot_face::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 5. Recognize finds shallow circular pocket ───────────────────────────
TEST(SkillSpotFace, RecognizeFindsSpotFace)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);
    skill::spot_face::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0; in.position_y_mm = 30.0;
    in.dia_mm = 12.0; in.depth_mm = 1.5;     // ratio = 0.125

    auto out = skill::spot_face::apply(*stock, in);
    auto cands = skill::spot_face::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["dia_mm"].get<double>(),   12.0, 0.1);
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(),  1.5, 0.1);
    EXPECT_GE(cands[0].confidence, 0.6);
}
