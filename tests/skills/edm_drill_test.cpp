// @lat: [[process/test-strategy#skill round-trip]]
//
// edm_drill — Small-hole EDM drilling round-trip tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/edm_drill.hpp"

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

// ── 1. Apply removes material at the EDM-drill envelope ──────────────────
TEST(SkillEdmDrill, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 15.0);

    skill::edm_drill::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 10.0;
    in.diameter_mm   = 0.5;
    in.depth_mm      = 12.0;     // ratio = 24 — within the warning band
    in.through_hole  = false;

    auto out = skill::edm_drill::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expected = M_PI * 0.25 * 0.25 * 12.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.05);
}

// ── 2. Validate rejects bad input (diameter > kMaxDia_mm) ────────────────
TEST(SkillEdmDrill, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 15.0);

    skill::edm_drill::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 10.0;
    in.diameter_mm   = 2.0;       // > kMaxDia_mm (1.0)
    in.depth_mm      = 5.0;

    auto r = skill::edm_drill::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-EDMD-DIA-HI" && f.severity == "error") {
            found = true; break;
        }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::edm_drill::apply(*stock, in), skill::SkillError);
}

// ── 3. Signature records kind + tool_type ────────────────────────────────
TEST(SkillEdmDrill, SignatureRecordsKindAndToolType)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 15.0);

    skill::edm_drill::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 10.0;
    in.diameter_mm   = 0.3;
    in.depth_mm      = 9.0;

    auto out = skill::edm_drill::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("edm_drill"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("edm_drill"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("edm_tube_electrode"));
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("brass"));
}

// ── 4. Recognize: metadata replay + geometric path ───────────────────────
TEST(SkillEdmDrill, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 15.0);

    skill::edm_drill::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 10.0;
    in.diameter_mm   = 0.4;
    in.depth_mm      = 10.0;     // ratio = 25 — within band

    auto out = skill::edm_drill::apply(*stock, in);
    auto cands = skill::edm_drill::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands[0].confidence, 0.8);
    EXPECT_NEAR(cands[0].recovered_params["diameter_mm"].get<double>(), 0.4, 1e-3);
}

// ── 5. Specific: depth/dia ratio > 50:1 rejected ─────────────────────────
TEST(SkillEdmDrill, DepthToDiaRatioEnforced)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 60.0);

    skill::edm_drill::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 10.0;
    in.diameter_mm   = 0.5;
    in.depth_mm      = 30.0;     // ratio = 60 — exceeds 50:1

    auto r = skill::edm_drill::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-EDMD-RATIO" && f.severity == "error") {
            found = true; break;
        }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::edm_drill::apply(*stock, in), skill::SkillError);
}
