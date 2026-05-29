// @lat: [[process/test-strategy#skill round-trip]]
//
// bullet_form skill — small-arms bullet seating no-op sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bullet_form.hpp"

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

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillBulletForm, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::bullet_form::Input in;
    in.caliber_mm          = 9.0;
    in.bullet_weight_grain = 115.0;
    in.jacket_material     = "copper";

    auto out = skill::bullet_form::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. ValidateRejectsBadInput: caliber out of range → error ────────────
TEST(SkillBulletForm, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::bullet_form::Input in;
    in.caliber_mm          = 0.5;     // below threshold
    in.bullet_weight_grain = 115.0;
    in.jacket_material     = "copper";

    auto r = skill::bullet_form::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::bullet_form::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillBulletForm, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::bullet_form::Input in;
    in.caliber_mm          = 5.56;
    in.bullet_weight_grain = 62.0;
    in.jacket_material     = "gilding_metal";

    auto out = skill::bullet_form::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("bullet_form"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("bullet_form"));
    EXPECT_NEAR(out.signature.pattern["caliber_mm"].get<double>(),
                5.56, 1e-6);
    EXPECT_NEAR(out.signature.pattern["bullet_weight_grain"].get<double>(),
                62.0, 1e-6);
    EXPECT_EQ(out.signature.pattern["jacket_material"].get<std::string>(),
              std::string("gilding_metal"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillBulletForm, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::bullet_form::Input in;
    in.caliber_mm          = 9.0;
    in.bullet_weight_grain = 124.0;
    in.jacket_material     = "copper";

    auto out  = skill::bullet_form::apply(*stock, in);
    auto cands = skill::bullet_form::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["caliber_mm"].get<double>(),
                9.0, 1e-6);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::bullet_form::recognize(raw).empty());
}

// ─── 5. Skill-specific: tool_type carries seating die ────────────────────
TEST(SkillBulletForm, ToolingDescribesSeatingDie)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::bullet_form::Input in;
    in.caliber_mm          = 11.43;   // .45 ACP
    in.bullet_weight_grain = 230.0;
    in.jacket_material     = "copper";

    auto out = skill::bullet_form::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("bullet_seating_die"));
    EXPECT_NEAR(out.signature.tooling.tool_dia_mm, 11.43, 1e-6);
}
