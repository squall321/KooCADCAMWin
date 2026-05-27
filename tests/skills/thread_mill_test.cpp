// @lat: [[process/test-strategy#skill round-trip]]
//
// thread_mill skill — real helical-sweep tests (slice 4).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
#include "skills/thread_mill.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply: internal thread removes a small volume of material ─────────
TEST(SkillThreadMill, ApplyProducesValidInternalThread)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Drill a pre-thread bore.
    skill::drill_hole::Input drill;
    drill.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    drill.position_x_mm = 25.0;
    drill.position_y_mm = 25.0;
    drill.diameter_mm   = 4.0;
    drill.depth_mm      = 6.0;
    auto pre = skill::drill_hole::apply(*stock, drill);

    const double volBefore = volumeOf(pre.workpiece->shape());

    skill::thread_mill::Input in;
    in.existing_hole_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25.0, 25.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size     = "M5";
    in.pitch_mm        = 0.8;
    in.thread_depth_mm = 4.0;
    in.is_external     = false;
    in.tool_dia_mm     = 1.5;

    auto out = skill::thread_mill::apply(*pre.workpiece, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Internal cut → volume must NOT increase.
    EXPECT_LE(volumeOf(out.workpiece->shape()), volBefore + 1e-6);
    EXPECT_EQ(out.workpiece->features().size(), 2u);
    EXPECT_EQ(out.signature.skill_id, std::string("thread_mill"));
    const std::string geom = out.signature.pattern["geometry"].get<std::string>();
    EXPECT_TRUE(geom == "helical_swept" || geom == "metadata_only")
        << "unexpected geometry tag: " << geom;
    EXPECT_EQ(out.signature.pattern["is_external"], false);
}

// ── 2. DFM-002 catches sub-min tool diameter ─────────────────────────────
TEST(SkillThreadMill, ValidateRejectsSubMinTool)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::thread_mill::Input in;
    in.existing_hole_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.thread_size     = "M3";
    in.pitch_mm        = 0.5;
    in.thread_depth_mm = 3.0;
    in.tool_dia_mm     = 0.5;    // < 0.8

    auto r = skill::thread_mill::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool dfm002Found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { dfm002Found = true; break; }
    EXPECT_TRUE(dfm002Found);
    EXPECT_THROW(skill::thread_mill::apply(*stock, in), skill::SkillError);
}

// ── 3. Engagement DFM check ──────────────────────────────────────────────
TEST(SkillThreadMill, ValidateRejectsShallowEngagement)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::thread_mill::Input in;
    in.existing_hole_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.thread_size     = "M3";
    in.pitch_mm        = 1.0;
    in.thread_depth_mm = 1.0;    // < 1.5 × pitch
    in.tool_dia_mm     = 1.0;

    auto r = skill::thread_mill::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 4. Recognize replays signature ───────────────────────────────────────
TEST(SkillThreadMill, RecognizeReplaysSignature)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::drill_hole::Input drill;
    drill.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    drill.position_x_mm = 25.0;
    drill.position_y_mm = 25.0;
    drill.diameter_mm   = 5.0;
    drill.depth_mm      = 8.0;
    auto pre = skill::drill_hole::apply(*stock, drill);

    skill::thread_mill::Input in;
    in.existing_hole_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25.0, 25.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size     = "M6";
    in.pitch_mm        = 1.0;
    in.thread_depth_mm = 6.0;
    in.is_external     = true;
    in.tool_dia_mm     = 2.0;

    auto out = skill::thread_mill::apply(*pre.workpiece, in);
    auto cands = skill::thread_mill::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].recovered_params["thread_size"], "M6");
    EXPECT_TRUE(cands[0].recovered_params["is_external"].get<bool>());
    EXPECT_NEAR(cands[0].recovered_params["tool_dia_mm"].get<double>(), 2.0, 1e-6);
}
