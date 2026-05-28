// @lat: [[process/test-strategy#skill round-trip]]
//
// plunge_roughing — pocket roughing by a grid of plunge drills + cleanup.
// Geometric result identical to mill_rect_pocket at slice-1 fidelity;
// metadata records the plunge grid.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/plunge_roughing.hpp"

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

// ── 1. Apply creates the pocket ──────────────────────────────────────────
TEST(SkillPlungeRoughing, ApplyCreatesPocket)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::plunge_roughing::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm        = 30.0;
    in.center_y_mm        = 20.0;
    in.length_mm          = 20.0;
    in.width_mm           = 10.0;
    in.depth_mm           = 3.0;
    in.corner_r_mm        = 1.0;
    in.tool_dia_mm        = 6.0;
    in.plunge_stepover_mm = 3.0;

    auto out = skill::plunge_roughing::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approx  = 20.0 * 10.0 * 3.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.10);

    EXPECT_EQ(out.signature.skill_id, std::string("plunge_roughing"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("plunge_roughing"));
}

// ── 2. DFM-002: sub-min tool rejected ────────────────────────────────────
TEST(SkillPlungeRoughing, ValidateRejectsSubMinTool)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::plunge_roughing::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm        = 25.0;
    in.center_y_mm        = 25.0;
    in.length_mm          = 10.0;
    in.width_mm           = 6.0;
    in.depth_mm           = 2.0;
    in.corner_r_mm        = 0.5;
    in.tool_dia_mm        = 0.5;
    in.plunge_stepover_mm = 0.3;

    auto r = skill::plunge_roughing::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 3. DFM-PLUNGE-STEP: stepover > tool_dia rejected ─────────────────────
TEST(SkillPlungeRoughing, ValidateRejectsHighStepover)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::plunge_roughing::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm        = 30.0;
    in.center_y_mm        = 20.0;
    in.length_mm          = 20.0;
    in.width_mm           = 10.0;
    in.depth_mm           = 3.0;
    in.corner_r_mm        = 1.0;
    in.tool_dia_mm        = 6.0;
    in.plunge_stepover_mm = 8.0;     // > tool_dia → error

    auto r = skill::plunge_roughing::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PLUNGE-STEP") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::plunge_roughing::apply(*stock, in), skill::SkillError);
}

// ── 4. Signature records the plunge grid + reduced cutting params ────────
TEST(SkillPlungeRoughing, SignatureRecordsPlungeGrid)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::plunge_roughing::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm        = 30.0;
    in.center_y_mm        = 20.0;
    in.length_mm          = 20.0;
    in.width_mm           = 10.0;
    in.depth_mm           = 3.0;
    in.corner_r_mm        = 1.0;
    in.tool_dia_mm        = 6.0;
    in.plunge_stepover_mm = 3.0;

    auto out = skill::plunge_roughing::apply(*stock, in);

    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("end_mill_plunge_capable"));
    // Plunge roughing uses slower cutting parameters than conventional.
    EXPECT_LE(out.signature.tooling.cutting_speed_sfm, 500.0);
    EXPECT_LE(out.signature.tooling.feed_per_tooth_mm, 0.02);

    const auto& pat = out.signature.pattern;
    ASSERT_TRUE(pat.contains("plunge_grid"));
    EXPECT_GT(pat.at("plunge_count").get<size_t>(), 0u);
    EXPECT_EQ(pat.at("plunge_grid").size(),
              pat.at("plunge_count").get<size_t>());
}

// ── 5. Default plunge_stepover = tool_dia × 0.6 ──────────────────────────
TEST(SkillPlungeRoughing, DefaultStepoverAutoFilled)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::plunge_roughing::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm        = 30.0;
    in.center_y_mm        = 20.0;
    in.length_mm          = 20.0;
    in.width_mm           = 10.0;
    in.depth_mm           = 2.0;
    in.corner_r_mm        = 1.0;
    in.tool_dia_mm        = 5.0;
    in.plunge_stepover_mm = 0.0;       // → 5.0 * 0.6 = 3.0

    auto out = skill::plunge_roughing::apply(*stock, in);
    EXPECT_NEAR(out.signature.params["plunge_stepover_mm"].get<double>(),
                3.0, 1e-6);
}

// ── 6. Recognize via feature history ─────────────────────────────────────
TEST(SkillPlungeRoughing, RecognizeFindsViaHistory)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::plunge_roughing::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm        = 30.0;
    in.center_y_mm        = 20.0;
    in.length_mm          = 20.0;
    in.width_mm           = 10.0;
    in.depth_mm           = 3.0;
    in.corner_r_mm        = 1.0;
    in.tool_dia_mm        = 6.0;
    in.plunge_stepover_mm = 3.0;

    auto out = skill::plunge_roughing::apply(*stock, in);
    auto cands = skill::plunge_roughing::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].skill_id, std::string("plunge_roughing"));
    EXPECT_GE(cands[0].confidence, 0.80);
}
