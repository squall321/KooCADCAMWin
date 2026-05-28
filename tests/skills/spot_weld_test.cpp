// @lat: [[process/test-strategy#skill round-trip]]
//
// spot_weld skill — 5-case sweep covering synthesis, DFM, recognition,
// round-trip and aspect-ratio enforcement.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/spot_weld.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply: bead fuses onto the top face (volume INCREASES) ─────────────
TEST(SkillSpotWeld, ApplyAddsBead)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::spot_weld::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.weld_dia_mm   = 6.0;
    in.weld_height_mm = 1.0;
    in.material      = "carbon_steel";

    auto out = skill::spot_weld::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume must INCREASE by ~ π × r² × h.
    const double approxAdded = M_PI * 3.0 * 3.0 * 1.0;
    const double volAfter = volumeOf(out.workpiece->shape());
    const double added = volAfter - volBefore;
    EXPECT_GT(added, approxAdded * 0.5)
        << "fuse should ADD ~π r² h volume; got " << added;
    EXPECT_LT(added, approxAdded * 1.5);

    EXPECT_EQ(out.signature.skill_id, std::string("spot_weld"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("resistance_welder"));
}

// ─── 2. DFM: too small / too large electrode tip rejected ─────────────────
TEST(SkillSpotWeld, ValidateRejectsBadDiameter)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Too small.
    skill::spot_weld::Input tooSmall;
    tooSmall.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tooSmall.position_x_mm = 25.0;
    tooSmall.position_y_mm = 25.0;
    tooSmall.weld_dia_mm   = 2.0;     // < 3
    tooSmall.weld_height_mm = 0.5;

    auto r1 = skill::spot_weld::validate(*stock, tooSmall);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::spot_weld::apply(*stock, tooSmall), skill::SkillError);

    // Too large.
    skill::spot_weld::Input tooLarge = tooSmall;
    tooLarge.weld_dia_mm   = 15.0;   // > 12
    tooLarge.weld_height_mm = 1.0;
    auto r2 = skill::spot_weld::validate(*stock, tooLarge);
    EXPECT_FALSE(r2.passed);
    EXPECT_THROW(skill::spot_weld::apply(*stock, tooLarge), skill::SkillError);
}

// ─── 3. DFM: height / dia ratio enforced ──────────────────────────────────
TEST(SkillSpotWeld, ValidateRejectsTallBead)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::spot_weld::Input tall;
    tall.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tall.position_x_mm = 25.0;
    tall.position_y_mm = 25.0;
    tall.weld_dia_mm   = 4.0;
    tall.weld_height_mm = 3.0;     // 3 / 4 > 0.4 → reject

    auto r = skill::spot_weld::validate(*stock, tall);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-WELD-RATIO") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Recognize: metadata replay finds the bead ─────────────────────────
TEST(SkillSpotWeld, RecognizeViaMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::spot_weld::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.weld_dia_mm   = 6.0;
    in.weld_height_mm = 1.0;

    auto out = skill::spot_weld::apply(*stock, in);
    auto cands = skill::spot_weld::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands[0].confidence, 0.9);  // metadata replay is full confidence
    EXPECT_NEAR(cands[0].recovered_params["weld_dia_mm"].get<double>(), 6.0, 1e-3);
    EXPECT_NEAR(cands[0].recovered_params["weld_height_mm"].get<double>(), 1.0, 1e-3);
}

// ─── 5. Recognize: geometric (workpiece without features) ─────────────────
TEST(SkillSpotWeld, RecognizeGeometric)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::spot_weld::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.weld_dia_mm   = 8.0;
    in.weld_height_mm = 1.5;

    auto out = skill::spot_weld::apply(*stock, in);

    // Recreate a feature-less workpiece from the same shape.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands = skill::spot_weld::recognize(raw);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands[0];
    EXPECT_NEAR(c.recovered_params["weld_dia_mm"].get<double>(), 8.0, 0.1);
    EXPECT_NEAR(c.recovered_params["weld_height_mm"].get<double>(), 1.5, 0.1);
}

// ─── 6. Multiple beads can be fused without collision ─────────────────────
TEST(SkillSpotWeld, MultipleBeads)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::spot_weld::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.weld_dia_mm   = 5.0;
    in.weld_height_mm = 0.8;
    in.position_x_mm = 15.0;
    in.position_y_mm = 30.0;
    auto w1 = skill::spot_weld::apply(*stock, in);

    in.position_x_mm = 45.0;
    auto w2 = skill::spot_weld::apply(*w1.workpiece, in);
    ASSERT_FALSE(w2.workpiece->shape().IsNull());
    EXPECT_EQ(w2.workpiece->features().size(), 2u);
}
