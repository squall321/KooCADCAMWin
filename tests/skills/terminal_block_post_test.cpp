// @lat: [[process/test-strategy#skill compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/terminal_block_post.hpp"

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

// 1. Apply (ADDITIVE) makes the volume GROW by the expected fused volume.
TEST(SkillTerminalBlockPost, ApplyFusesRealGeometry)
{
    // Insulator base: thin plate.
    auto stock = skill::createCuboidStock(30.0, 30.0, 3.0);
    const double volBefore = volumeOf(stock->shape());
    const int    facesBefore = stock->faceCount();

    skill::terminal_block_post::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 15.0;
    in.position_y_mm     = 15.0;
    in.stud_dia_mm       = 3.0;
    in.stud_height_mm    = 8.0;
    in.shoulder_dia_mm   = 5.0;
    in.shoulder_height_mm = 1.5;
    in.skirt_dia_mm      = 8.0;
    in.skirt_height_mm   = 5.0;
    in.wire_hole_dia_mm  = 1.6;
    in.wire_hole_z_mm    = 2.5;

    auto out = skill::terminal_block_post::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Net added volume = stud + shoulder + skirt − 2×(wire hole through skirt).
    const double rStud = 1.5;
    const double rSh   = 2.5;
    const double rSk   = 4.0;
    const double rWire = 0.8;
    // Each wire hole crosses the full skirt diameter = 2*rSk.
    const double approxAdd =
        M_PI * rStud * rStud * 8.0
      + M_PI * rSh   * rSh   * 1.5
      + M_PI * rSk   * rSk   * 5.0
      - 2.0 * M_PI * rWire * rWire * (2.0 * rSk);
    const double diff = volumeOf(out.workpiece->shape()) - volBefore;
    EXPECT_NEAR(diff, approxAdd, approxAdd * 0.20)
        << "terminal_block_post should add stud+shoulder+skirt and remove wire holes";
    EXPECT_GT(out.workpiece->faceCount(), facesBefore);
}

// 2. Validate rejects bad input + apply throws.
TEST(SkillTerminalBlockPost, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 3.0);
    skill::terminal_block_post::Input bad;
    bad.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.position_x_mm      = 15.0;
    bad.position_y_mm      = 15.0;
    bad.stud_dia_mm        = 1.5;     // < M2 — DFM-IEC60947
    bad.stud_height_mm     = 8.0;
    bad.shoulder_dia_mm    = 5.0;
    bad.shoulder_height_mm = 1.5;
    bad.skirt_dia_mm       = 8.0;
    bad.skirt_height_mm    = 5.0;
    bad.wire_hole_dia_mm   = 1.6;
    bad.wire_hole_z_mm     = 2.5;

    auto r = skill::terminal_block_post::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-IEC60947") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::terminal_block_post::apply(*stock, bad), skill::SkillError);
}

// 3. Signature: compound + subfeature_count >= 2.
TEST(SkillTerminalBlockPost, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 3.0);
    skill::terminal_block_post::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 15.0;
    in.position_y_mm      = 15.0;
    in.stud_dia_mm        = 3.0;
    in.stud_height_mm     = 8.0;
    in.shoulder_dia_mm    = 5.0;
    in.shoulder_height_mm = 1.5;
    in.skirt_dia_mm       = 8.0;
    in.skirt_height_mm    = 5.0;
    in.wire_hole_dia_mm   = 1.6;
    in.wire_hole_z_mm     = 2.5;

    auto out = skill::terminal_block_post::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_TRUE(pat["is_compound"].get<bool>());
    EXPECT_GE(pat["subfeature_count"].get<int>(), 2);
    EXPECT_EQ(pat["wire_hole_count"].get<int>(), 2);

    const auto& p = out.signature.params;
    EXPECT_DOUBLE_EQ(p["stud_dia_mm"].get<double>(), 3.0);
    EXPECT_DOUBLE_EQ(p["skirt_dia_mm"].get<double>(), 8.0);
}

// 4. Recognize: metadata replay confidence 1.0; geometric ≥0.5 when present.
TEST(SkillTerminalBlockPost, RecognizeReturnsCandidate)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 3.0);
    skill::terminal_block_post::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 15.0;
    in.position_y_mm      = 15.0;
    in.stud_dia_mm        = 3.0;
    in.stud_height_mm     = 8.0;
    in.shoulder_dia_mm    = 5.0;
    in.shoulder_height_mm = 1.5;
    in.skirt_dia_mm       = 8.0;
    in.skirt_height_mm    = 5.0;
    in.wire_hole_dia_mm   = 1.6;
    in.wire_hole_z_mm     = 2.5;

    auto out = skill::terminal_block_post::apply(*stock, in);
    auto cands = skill::terminal_block_post::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    bool hasMeta = false;
    for (const auto& c : cands)
        if (std::abs(c.confidence - 1.0) < 1e-6) { hasMeta = true; break; }
    EXPECT_TRUE(hasMeta);
}

// 5. Feature-specific: skirt OD strictly > shoulder OD > stud OD.
TEST(SkillTerminalBlockPost, FeatureSpecific_OrderedDiameters)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 3.0);
    skill::terminal_block_post::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 15.0;
    in.position_y_mm      = 15.0;
    in.stud_dia_mm        = 3.0;
    in.stud_height_mm     = 8.0;
    in.shoulder_dia_mm    = 5.0;
    in.shoulder_height_mm = 1.5;
    in.skirt_dia_mm       = 8.0;
    in.skirt_height_mm    = 5.0;
    in.wire_hole_dia_mm   = 1.6;
    in.wire_hole_z_mm     = 2.5;

    auto out = skill::terminal_block_post::apply(*stock, in);
    const auto& p = out.signature.params;
    EXPECT_LT(p["stud_dia_mm"].get<double>(),
              p["shoulder_dia_mm"].get<double>());
    EXPECT_LT(p["shoulder_dia_mm"].get<double>(),
              p["skirt_dia_mm"].get<double>());
}
