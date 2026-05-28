// @lat: [[process/test-strategy#skill round-trip]]
//
// 3d_print — additive manufacturing process skill (no geometric change).
// Covers FDM, SLA, SLS, and metal-laser process types.
//
// Cases (7):
//   1. FDM apply is a geometric no-op + signature carries metadata.
//   2. SLA apply does NOT carry infill_pct (infill_pct → 0 in signature).
//   3. DFM-PRINT-LAYER rejects layer_height outside [0.05, 0.5].
//   4. DFM-PRINT-INFILL rejects out-of-range FDM infill.
//   5. DFM-INPUT rejects unknown process_type.
//   6. Material/process mismatch yields an INFO (still passes).
//   7. Recognize replays history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/3d_print.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. FDM apply: no geometric change, metadata recorded ──────────────────
TEST(Skill3DPrint, FdmApplyIsGeometricNoOp)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::p3d_print::Input in;
    in.process_type    = "fdm";
    in.layer_height_mm = 0.2;
    in.material        = "pla";
    in.infill_pct      = 25.0;

    auto out = skill::p3d_print::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);

    EXPECT_EQ(out.signature.skill_id, std::string("3d_print"));
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["process_supports_infill"].get<bool>());
    EXPECT_NEAR(out.signature.pattern["infill_pct"].get<double>(), 25.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["process_subtype"].get<std::string>(),
              std::string("fdm"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("fdm_extruder"));
}

// ─── 2. SLA: infill_pct ignored ────────────────────────────────────────────
TEST(Skill3DPrint, SlaIgnoresInfill)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::p3d_print::Input in;
    in.process_type    = "sla";
    in.layer_height_mm = 0.05;
    in.material        = "resin";
    in.infill_pct      = 50.0;        // ignored for SLA

    auto out = skill::p3d_print::apply(*stock, in);

    EXPECT_FALSE(out.signature.pattern["process_supports_infill"].get<bool>());
    EXPECT_NEAR(out.signature.params["infill_pct"].get<double>(), 0.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["infill_pct"].get<double>(), 0.0, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("sla_uv_projector"));
}

// ─── 3. DFM rejects layer height outside [0.05, 0.5] ───────────────────────
TEST(Skill3DPrint, ValidateRejectsBadLayerHeight)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::p3d_print::Input tooThin;
    tooThin.process_type    = "fdm";
    tooThin.layer_height_mm = 0.01;        // < 0.05
    tooThin.material        = "pla";
    tooThin.infill_pct      = 20.0;
    {
        auto r = skill::p3d_print::validate(*stock, tooThin);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-PRINT-LAYER") { found = true; break; }
        EXPECT_TRUE(found);
    }

    skill::p3d_print::Input tooThick = tooThin;
    tooThick.layer_height_mm = 0.8;        // > 0.5
    {
        auto r = skill::p3d_print::validate(*stock, tooThick);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-PRINT-LAYER") { found = true; break; }
        EXPECT_TRUE(found);
    }

    EXPECT_THROW(skill::p3d_print::apply(*stock, tooThin), skill::SkillError);
}

// ─── 4. DFM rejects FDM infill outside [10, 100] ───────────────────────────
TEST(Skill3DPrint, ValidateRejectsBadFdmInfill)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::p3d_print::Input lowInfill;
    lowInfill.process_type    = "fdm";
    lowInfill.layer_height_mm = 0.2;
    lowInfill.material        = "pla";
    lowInfill.infill_pct      = 5.0;        // < 10
    auto r = skill::p3d_print::validate(*stock, lowInfill);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PRINT-INFILL") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::p3d_print::apply(*stock, lowInfill), skill::SkillError);
}

// ─── 5. DFM rejects unknown process_type ───────────────────────────────────
TEST(Skill3DPrint, ValidateRejectsUnknownProcess)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::p3d_print::Input in;
    in.process_type    = "bioprint";    // unknown
    in.layer_height_mm = 0.2;
    in.material        = "pla";
    in.infill_pct      = 20.0;

    auto r = skill::p3d_print::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::p3d_print::apply(*stock, in), skill::SkillError);
}

// ─── 6. Material/process mismatch yields INFO (still passes) ───────────────
TEST(Skill3DPrint, MaterialProcessMismatchYieldsInfo)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    // PLA on SLA — PLA is for FDM, so process planner should warn (info).
    skill::p3d_print::Input in;
    in.process_type    = "sla";
    in.layer_height_mm = 0.1;
    in.material        = "pla";       // not typical SLA material
    in.infill_pct      = 0.0;

    auto r = skill::p3d_print::validate(*stock, in);
    EXPECT_TRUE(r.passed);            // info, not error
    bool sawCompat = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PRINT-COMPAT" && f.severity == "info") {
            sawCompat = true;
            break;
        }
    EXPECT_TRUE(sawCompat);
}

// ─── 7. Recognize replays history ──────────────────────────────────────────
TEST(Skill3DPrint, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::p3d_print::Input in;
    in.process_type    = "metal_laser";
    in.layer_height_mm = 0.06;          // inside [0.05, 0.5] DFM band
    in.material        = "stainless_316l";
    in.infill_pct      = 0.0;           // ignored for non-FDM

    auto out   = skill::p3d_print::apply(*stock, in);
    auto cands = skill::p3d_print::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_GT(cands[0].confidence, 0.99);
    EXPECT_EQ(cands[0].recovered_params["process_type"].get<std::string>(),
              std::string("metal_laser"));
    EXPECT_NEAR(cands[0].recovered_params["layer_height_mm"].get<double>(),
                0.06, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["material"].get<std::string>(),
              std::string("stainless_316l"));
}
