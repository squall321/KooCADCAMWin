// @lat: [[process/test-strategy#skill round-trip]]
//
// wood_planing skill — surface planing reduces thickness.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/wood_planing.hpp"

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

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

}  // namespace

// ─── 1. Apply handles input: 20mm board → 19.2mm after 0.8mm pass ────────
TEST(SkillWoodPlaning, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 20.0, "oak");
    EXPECT_NEAR(skill::wood_planing::currentThickness(*stock), 20.0, 1e-6);

    skill::wood_planing::Input in;
    in.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.removal_mm  = 0.8;

    auto out = skill::wood_planing::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Z (smallest axis) shrinks; X/Y preserved.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    out.workpiece->boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    EXPECT_NEAR(xMax - xMin, 200.0, 1e-6);
    EXPECT_NEAR(yMax - yMin, 100.0, 1e-6);
    EXPECT_NEAR(zMax - zMin, 19.2,  1e-6);

    EXPECT_EQ(out.signature.skill_id, std::string("wood_planing"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_LT(volumeOf(out.workpiece->shape()), volumeOf(stock->shape()));
}

// ─── 2. Validate rejects bad input (too aggressive / too thin) ───────────
TEST(SkillWoodPlaning, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 20.0);

    // Zero removal → DFM-INPUT.
    skill::wood_planing::Input zero;
    zero.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    zero.removal_mm = 0.0;
    auto r1 = skill::wood_planing::validate(*stock, zero);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-INPUT"));

    // > 3 mm single pass → DFM-PLANE-PASS.
    skill::wood_planing::Input aggressive;
    aggressive.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    aggressive.removal_mm = 5.0;
    auto r2 = skill::wood_planing::validate(*stock, aggressive);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-PLANE-PASS"));
    EXPECT_THROW(skill::wood_planing::apply(*stock, aggressive), skill::SkillError);

    // Leaves <6 mm → DFM-PLANE-LEAVE.
    auto thinStock = skill::createCuboidStock(200.0, 100.0, 6.5);
    skill::wood_planing::Input leave;
    leave.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    leave.removal_mm = 1.0;
    auto r3 = skill::wood_planing::validate(*thinStock, leave);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-PLANE-LEAVE"));
}

// ─── 3. Signature records kind + removal_mm + thickness axis ─────────────
TEST(SkillWoodPlaning, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 25.0);

    skill::wood_planing::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.removal_mm = 1.5;

    auto out = skill::wood_planing::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), std::string("wood_planing"));
    EXPECT_NEAR(out.signature.pattern["removal_mm"].get<double>(), 1.5, 1e-6);
    EXPECT_EQ(out.signature.pattern["thickness_axis"].get<std::string>(), std::string("z"));
    EXPECT_NEAR(out.signature.pattern["original_thickness_mm"].get<double>(), 25.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["target_thickness_mm"].get<double>(),   23.5, 1e-6);
    EXPECT_EQ(out.signature.pattern["surface_finish"].get<std::string>(), std::string("smooth"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("wood_planer_head"));
    EXPECT_GT(out.signature.tooling.stock_removed_mm3, 0.0);
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillWoodPlaning, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 20.0);

    skill::wood_planing::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.removal_mm = 1.0;
    auto out = skill::wood_planing::apply(*stock, in);

    auto cands = skill::wood_planing::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["removal_mm"].get<double>(), 1.0, 1e-6);

    // Bare workpiece → geometric fallback (low confidence).
    skill::Workpiece bare(out.workpiece->shape(), out.workpiece->material());
    auto cands2 = skill::wood_planing::recognize(bare);
    ASSERT_FALSE(cands2.empty());
    EXPECT_LT(cands2[0].confidence, 0.5);
}

// ─── 5. Volume removed matches expected shaving (footprint × removal) ────
TEST(SkillWoodPlaning, VolumeRemovedMatchesShaving)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 30.0);
    const double volBefore = volumeOf(stock->shape());

    skill::wood_planing::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.removal_mm = 2.0;

    auto out = skill::wood_planing::apply(*stock, in);
    const double volAfter = volumeOf(out.workpiece->shape());
    const double expected = 200.0 * 100.0 * 2.0;
    EXPECT_NEAR(volBefore - volAfter, expected, expected * 0.001);
}
