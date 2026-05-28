// @lat: [[process/test-strategy#skill round-trip]]
//
// fused_dep_model — FDM with explicit nozzle geometry (no geometric change).
//
// Cases (7):
//   1. Apply is a geometric no-op + signature records nozzle/layer/material.
//   2. layer_to_nozzle_ratio is computed in the pattern.
//   3. DFM-FDM-LAYER rejects layer_height > nozzle × 0.8.
//   4. DFM-FDM-NOZ rejects nozzle outside [0.2, 1.0] mm.
//   5. DFM-FDM-SPEED rejects non-positive speeds.
//   6. Material can be specified and is preserved in signature.
//   7. Recognize replays history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/fused_dep_model.hpp"

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

// ─── 1. Apply: no geometric change + signature ─────────────────────────────
TEST(SkillFusedDepModel, ApplyIsGeometricNoOp)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    const double volBefore = volumeOf(stock->shape());

    skill::fused_dep_model::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.nozzle_dia_mm        = 0.4;
    in.layer_height_mm      = 0.2;
    in.material             = "pla";
    in.print_speed_mm_per_s = 60.0;

    auto out = skill::fused_dep_model::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);

    EXPECT_EQ(out.signature.skill_id, std::string("fused_dep_model"));
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_EQ(out.signature.pattern["process_subtype"].get<std::string>(),
              std::string("fdm"));
    EXPECT_EQ(out.signature.pattern["process_alias"].get<std::string>(),
              std::string("fused_deposition_modeling"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("fdm_nozzle"));
    EXPECT_NEAR(out.signature.tooling.tool_dia_mm, 0.4, 1e-9);
}

// ─── 2. layer_to_nozzle_ratio is computed ──────────────────────────────────
TEST(SkillFusedDepModel, ComputesLayerNozzleRatio)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::fused_dep_model::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.nozzle_dia_mm        = 0.8;
    in.layer_height_mm      = 0.4;     // 0.4 / 0.8 = 0.5
    in.material             = "petg";
    in.print_speed_mm_per_s = 40.0;

    auto out = skill::fused_dep_model::apply(*stock, in);

    EXPECT_NEAR(out.signature.pattern["layer_to_nozzle_ratio"].get<double>(),
                0.5, 1e-9);
}

// ─── 3. DFM rejects layer_height > nozzle × 0.8 ────────────────────────────
TEST(SkillFusedDepModel, ValidateRejectsBadLayerNozzleRatio)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::fused_dep_model::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.nozzle_dia_mm        = 0.4;
    in.layer_height_mm      = 0.35;    // 0.35 / 0.4 = 0.875 > 0.8 → reject
    in.material             = "pla";
    in.print_speed_mm_per_s = 60.0;

    auto r = skill::fused_dep_model::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FDM-LAYER" && f.severity == "error") {
            found = true;
            break;
        }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::fused_dep_model::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects nozzle outside [0.2, 1.0] mm ───────────────────────────
TEST(SkillFusedDepModel, ValidateRejectsBadNozzleDia)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::fused_dep_model::Input tooSmall;
    tooSmall.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tooSmall.nozzle_dia_mm        = 0.1;          // < 0.2
    tooSmall.layer_height_mm      = 0.05;
    tooSmall.material             = "pla";
    tooSmall.print_speed_mm_per_s = 30.0;
    {
        auto r = skill::fused_dep_model::validate(*stock, tooSmall);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-FDM-NOZ") { found = true; break; }
        EXPECT_TRUE(found);
    }

    skill::fused_dep_model::Input tooBig;
    tooBig.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tooBig.nozzle_dia_mm        = 1.5;            // > 1.0
    tooBig.layer_height_mm      = 0.5;
    tooBig.material             = "pla";
    tooBig.print_speed_mm_per_s = 30.0;
    {
        auto r = skill::fused_dep_model::validate(*stock, tooBig);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-FDM-NOZ") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ─── 5. DFM rejects non-positive speeds ────────────────────────────────────
TEST(SkillFusedDepModel, ValidateRejectsZeroSpeed)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::fused_dep_model::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.nozzle_dia_mm        = 0.4;
    in.layer_height_mm      = 0.2;
    in.material             = "pla";
    in.print_speed_mm_per_s = 0.0;           // invalid

    auto r = skill::fused_dep_model::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FDM-SPEED" && f.severity == "error") {
            found = true;
            break;
        }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::fused_dep_model::apply(*stock, in), skill::SkillError);
}

// ─── 6. Material preserved in signature ────────────────────────────────────
TEST(SkillFusedDepModel, MaterialPreservedInSignature)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::fused_dep_model::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.nozzle_dia_mm        = 0.6;
    in.layer_height_mm      = 0.3;
    in.material             = "carbon_fiber_nylon";   // arbitrary string
    in.print_speed_mm_per_s = 45.0;

    auto out = skill::fused_dep_model::apply(*stock, in);

    EXPECT_EQ(out.signature.params["material"].get<std::string>(),
              std::string("carbon_fiber_nylon"));
    EXPECT_NEAR(out.signature.params["nozzle_dia_mm"].get<double>(),       0.6, 1e-9);
    EXPECT_NEAR(out.signature.params["layer_height_mm"].get<double>(),     0.3, 1e-9);
    EXPECT_NEAR(out.signature.params["print_speed_mm_per_s"].get<double>(), 45.0, 1e-9);
}

// ─── 7. Recognize replays history ──────────────────────────────────────────
TEST(SkillFusedDepModel, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::fused_dep_model::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.nozzle_dia_mm        = 0.5;
    in.layer_height_mm      = 0.25;
    in.material             = "abs";
    in.print_speed_mm_per_s = 50.0;

    auto out   = skill::fused_dep_model::apply(*stock, in);
    auto cands = skill::fused_dep_model::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_GT(cands[0].confidence, 0.99);
    EXPECT_NEAR(cands[0].recovered_params["nozzle_dia_mm"].get<double>(), 0.5, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["layer_height_mm"].get<double>(),
                0.25, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["material"].get<std::string>(),
              std::string("abs"));

    // recognize on a feature-less stock returns nothing.
    auto bare = skill::createCuboidStock(40.0, 40.0, 8.0);
    EXPECT_TRUE(skill::fused_dep_model::recognize(*bare).empty());
}
