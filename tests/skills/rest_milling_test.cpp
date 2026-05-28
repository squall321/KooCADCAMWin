// @lat: [[process/test-strategy#skill round-trip]]
//
// rest_milling — corner-cleanup operation after a larger roughing tool.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rest_milling.hpp"

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

// ── 1. Apply removes the volume of 4 small corner drills ─────────────────
TEST(SkillRestMilling, ApplyCleansCorners)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::rest_milling::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm           = 30.0;
    in.center_y_mm           = 20.0;
    in.length_mm             = 20.0;
    in.width_mm              = 10.0;
    in.depth_mm              = 3.0;
    in.previous_tool_dia_mm  = 6.0;
    in.cleanup_tool_dia_mm   = 2.0;

    auto out = skill::rest_milling::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double r = 1.0;     // cleanup_tool_dia / 2
    const double expected = 4.0 * M_PI * r * r * 3.0;   // ~37.7 mm³
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.10);

    EXPECT_EQ(out.signature.skill_id, std::string("rest_milling"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("rest_milling"));
}

// ── 2. DFM-REST-DIA: cleanup >= previous rejected ────────────────────────
TEST(SkillRestMilling, ValidateRejectsCleanupNotSmaller)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::rest_milling::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm           = 25.0;
    in.center_y_mm           = 25.0;
    in.length_mm             = 12.0;
    in.width_mm              = 8.0;
    in.depth_mm              = 2.0;
    in.previous_tool_dia_mm  = 4.0;
    in.cleanup_tool_dia_mm   = 4.0;     // same as previous → error

    auto r = skill::rest_milling::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-REST-DIA") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::rest_milling::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-002: cleanup < 0.8 mm rejected ────────────────────────────────
TEST(SkillRestMilling, ValidateRejectsSubMinCleanup)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::rest_milling::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm           = 25.0;
    in.center_y_mm           = 25.0;
    in.length_mm             = 12.0;
    in.width_mm              = 8.0;
    in.depth_mm              = 2.0;
    in.previous_tool_dia_mm  = 6.0;
    in.cleanup_tool_dia_mm   = 0.5;

    auto r = skill::rest_milling::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. Signature records both tool diameters + corner positions ──────────
TEST(SkillRestMilling, SignatureRecordsToolDiametersAndCorners)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::rest_milling::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm           = 30.0;
    in.center_y_mm           = 20.0;
    in.length_mm             = 20.0;
    in.width_mm              = 10.0;
    in.depth_mm              = 3.0;
    in.previous_tool_dia_mm  = 6.0;
    in.cleanup_tool_dia_mm   = 1.5;

    auto out = skill::rest_milling::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_NEAR(pat.at("previous_tool_dia_mm").get<double>(), 6.0, 1e-6);
    EXPECT_NEAR(pat.at("cleanup_tool_dia_mm" ).get<double>(), 1.5, 1e-6);
    ASSERT_TRUE(pat.contains("corner_positions"));
    EXPECT_EQ(pat.at("corner_positions").size(), 4u);
}

// ── 5. Recognize via feature history ─────────────────────────────────────
TEST(SkillRestMilling, RecognizeFindsViaHistory)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::rest_milling::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm           = 30.0;
    in.center_y_mm           = 20.0;
    in.length_mm             = 20.0;
    in.width_mm              = 10.0;
    in.depth_mm              = 3.0;
    in.previous_tool_dia_mm  = 6.0;
    in.cleanup_tool_dia_mm   = 1.5;

    auto out = skill::rest_milling::apply(*stock, in);
    auto cands = skill::rest_milling::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].skill_id, std::string("rest_milling"));
    EXPECT_GE(cands[0].confidence, 0.80);
    EXPECT_NEAR(
        cands[0].recovered_params["cleanup_tool_dia_mm"].get<double>(),
        1.5, 1e-6);
}
