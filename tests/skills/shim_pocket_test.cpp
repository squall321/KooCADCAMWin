// @lat: [[process/test-strategy#compound macro]]
//
// shim_pocket — 4 sub-features:
//   main pocket + retention edge step + 2 retention tab notches.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/shim_pocket.hpp"

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

// ─── 1. apply produces a valid shape with multiple sub-features ───────
TEST(SkillShimPocket, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 15.0);
    const int faces0 = stock->faceCount();

    skill::shim_pocket::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.shim_thk_mm   = 1.0;
    in.shim_w_mm     = 10.0;
    in.shim_l_mm     = 20.0;
    in.retention_step_depth_mm = 0.1;
    in.retention_step_inset_mm = 1.0;
    in.tab_notch_w_mm = 1.5;
    in.tab_notch_d_mm = 0.8;

    const double v0 = volumeOf(stock->shape());
    auto out = skill::shim_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Volume removed > main pocket alone (step + tabs add to total).
    const double mainOnly = (20.0 + 0.1) * (10.0 + 0.1) * (1.0 + 0.05);
    EXPECT_GT(v0 - v1, mainOnly * 0.9);   // at least near full main pocket
    EXPECT_GT(out.workpiece->faceCount(), faces0);
}

// ─── 2. validate rejects bad input ────────────────────────────────────
TEST(SkillShimPocket, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 15.0);

    skill::shim_pocket::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.shim_thk_mm   = 10.0;     // > 5 mm range
    in.shim_w_mm     = 10.0;
    in.shim_l_mm     = 20.0;
    in.tab_notch_w_mm = 1.0;
    in.tab_notch_d_mm = 0.5;
    in.retention_step_depth_mm = 0.1;
    in.retention_step_inset_mm = 0.5;

    auto r = skill::shim_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SHIM-THK") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::shim_pocket::apply(*stock, in), skill::SkillError);
}

// ─── 3. signature records compound kind ───────────────────────────────
TEST(SkillShimPocket, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 15.0);

    skill::shim_pocket::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.shim_thk_mm   = 0.5;
    in.shim_w_mm     = 8.0;
    in.shim_l_mm     = 16.0;
    in.retention_step_depth_mm = 0.1;
    in.retention_step_inset_mm = 0.8;
    in.tab_notch_w_mm = 1.0;
    in.tab_notch_d_mm = 0.5;

    auto out = skill::shim_pocket::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("shim_pocket"));
    EXPECT_EQ(out.signature.pattern.at("kind"), std::string("shim_pocket"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
    EXPECT_NEAR(out.signature.params.at("shim_thk_mm").get<double>(),
                0.5, 1e-9);
}

// ─── 4. recognize via metadata replay ─────────────────────────────────
TEST(SkillShimPocket, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 15.0);

    skill::shim_pocket::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.shim_thk_mm   = 2.0;
    in.shim_w_mm     = 12.0;
    in.shim_l_mm     = 18.0;
    in.retention_step_depth_mm = 0.15;
    in.retention_step_inset_mm = 1.2;
    in.tab_notch_w_mm = 2.0;
    in.tab_notch_d_mm = 1.0;

    auto out = skill::shim_pocket::apply(*stock, in);
    auto cands = skill::shim_pocket::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["shim_thk_mm"].get<double>(),
                2.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["shim_w_mm"].get<double>(),
                12.0, 1e-9);
}

// ─── 5. SPECIFIC: retention step footprint = main pocket - 2 × inset ──
TEST(SkillShimPocket, RetentionStepSmallerThanMainPocket)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 15.0);

    skill::shim_pocket::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.shim_thk_mm   = 1.0;
    in.shim_w_mm     = 10.0;
    in.shim_l_mm     = 20.0;
    in.retention_step_depth_mm = 0.1;
    in.retention_step_inset_mm = 0.5;
    in.tab_notch_w_mm = 1.0;
    in.tab_notch_d_mm = 0.5;

    auto out = skill::shim_pocket::apply(*stock, in);
    const double pocketL = out.signature.pattern.at("pocket_l_mm").get<double>();
    const double pocketW = out.signature.pattern.at("pocket_w_mm").get<double>();
    const double stepL   = out.signature.pattern.at("retention_step_l_mm").get<double>();
    const double stepW   = out.signature.pattern.at("retention_step_w_mm").get<double>();

    EXPECT_NEAR(pocketL - stepL, 2.0 * in.retention_step_inset_mm, 1e-6);
    EXPECT_NEAR(pocketW - stepW, 2.0 * in.retention_step_inset_mm, 1e-6);
    EXPECT_LT(stepL, pocketL);
    EXPECT_LT(stepW, pocketW);
}
