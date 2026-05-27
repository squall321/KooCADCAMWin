// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/undercut_milling.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}

TEST(SkillUndercutMilling, ApplyCreatesUndercut)
{
    // Use a thick block so the undercut pocket fits comfortably inside.
    auto stock = skill::createCuboidStock(80.0, 40.0, 30.0);

    skill::undercut_milling::Input in;
    // Enter through the +X face (normal = +X) of the block.
    in.entry_face        = skill::FaceByNormal{ gp_Dir(1, 0, 0) };
    in.position_y_mm     = 0.0;      // centered on face
    in.position_z_mm     = 0.0;
    in.pocket_width_mm   = 10.0;
    in.pocket_depth_mm   = 5.0;
    in.pocket_height_mm  = 8.0;

    auto out = skill::undercut_milling::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Box volume = width × depth × height
    const double approxRem = 10.0 * 5.0 * 8.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approxRem, approxRem * 0.10);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("undercut_milling"));
}

TEST(SkillUndercutMilling, ValidateAlwaysEmitsInfoFinding)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 30.0);
    skill::undercut_milling::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(1, 0, 0) };
    in.position_y_mm     = 0.0;
    in.position_z_mm     = 0.0;
    in.pocket_width_mm   = 10.0;
    in.pocket_depth_mm   = 5.0;
    in.pocket_height_mm  = 8.0;

    auto r = skill::undercut_milling::validate(*stock, in);
    // The DFM-005 info-finding must always be present, even when passed.
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-005" && f.severity == "info") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_TRUE(r.passed);  // info findings do not fail validation
}

TEST(SkillUndercutMilling, ValidateRejectsSubMin)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 30.0);
    skill::undercut_milling::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(1, 0, 0) };
    in.position_y_mm     = 0.0;
    in.position_z_mm     = 0.0;
    in.pocket_width_mm   = 0.5;       // < 0.8 → DFM-002 error
    in.pocket_depth_mm   = 5.0;
    in.pocket_height_mm  = 8.0;

    auto r = skill::undercut_milling::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::undercut_milling::apply(*stock, in), skill::SkillError);
}

TEST(SkillUndercutMilling, SignatureMarksAsUndercut)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 30.0);
    skill::undercut_milling::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(1, 0, 0) };
    in.position_y_mm     = 0.0;
    in.position_z_mm     = 0.0;
    in.pocket_width_mm   = 10.0;
    in.pocket_depth_mm   = 5.0;
    in.pocket_height_mm  = 8.0;

    auto out = skill::undercut_milling::apply(*stock, in);
    // The pattern must mark this as non-top-accessible.
    EXPECT_FALSE(out.signature.pattern["is_top_accessible"].get<bool>());
    // The tooling extra carries a 5-axis constraint note.
    EXPECT_TRUE(out.signature.tooling.extra.contains("machining_constraint"));
}

TEST(SkillUndercutMilling, RecognizeFinds)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 30.0);
    skill::undercut_milling::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(1, 0, 0) };
    in.position_y_mm     = 0.0;
    in.position_z_mm     = 0.0;
    in.pocket_width_mm   = 10.0;
    in.pocket_depth_mm   = 5.0;
    in.pocket_height_mm  = 8.0;

    auto out = skill::undercut_milling::apply(*stock, in);
    auto cands = skill::undercut_milling::recognize(*out.workpiece);
    // Lower-confidence heuristic; we just verify it returns a candidate.
    ASSERT_FALSE(cands.empty());
    EXPECT_GE(cands[0].confidence, 0.5);
    EXPECT_LE(cands[0].confidence, 0.7);
}
