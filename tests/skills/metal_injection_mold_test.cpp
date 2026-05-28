// @lat: [[process/test-strategy#skill round-trip]]
//
// metal_injection_mold — MIM process skill (no geometric change).
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKind + key params
//   4. RecognizeMetadataReplay
//   5. Predicted cavity scale derives from sinter_shrinkage_pct

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/metal_injection_mold.hpp"

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

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillMetalInjectionMold, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);
    const double volBefore   = volumeOf(stock->shape());
    const int    facesBefore = stock->faceCount();
    const int    edgesBefore = stock->edgeCount();

    skill::metal_injection_mold::Input in;
    in.material             = "stainless_316";
    in.binder_pct           = 40.0;
    in.sinter_shrinkage_pct = 17.0;
    in.wall_thickness_mm    = 2.0;

    auto out = skill::metal_injection_mold::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_EQ(out.workpiece->faceCount(), facesBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgesBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillMetalInjectionMold, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    // Sub-MIM-minimum wall thickness.
    skill::metal_injection_mold::Input thinWall;
    thinWall.material             = "stainless_316";
    thinWall.binder_pct           = 40.0;
    thinWall.sinter_shrinkage_pct = 17.0;
    thinWall.wall_thickness_mm    = 0.2;
    auto rWall = skill::metal_injection_mold::validate(*stock, thinWall);
    EXPECT_FALSE(rWall.passed);
    EXPECT_THROW(skill::metal_injection_mold::apply(*stock, thinWall),
                 skill::SkillError);

    // Binder out of range.
    skill::metal_injection_mold::Input badBinder;
    badBinder.material             = "stainless_316";
    badBinder.binder_pct           = 5.0;          // < 30
    badBinder.sinter_shrinkage_pct = 17.0;
    badBinder.wall_thickness_mm    = 2.0;
    auto rBinder = skill::metal_injection_mold::validate(*stock, badBinder);
    EXPECT_FALSE(rBinder.passed);

    // Shrinkage out of (0, 100).
    skill::metal_injection_mold::Input badShrink;
    badShrink.material             = "stainless_316";
    badShrink.binder_pct           = 40.0;
    badShrink.sinter_shrinkage_pct = 0.0;          // invalid
    badShrink.wall_thickness_mm    = 2.0;
    auto rShrink = skill::metal_injection_mold::validate(*stock, badShrink);
    EXPECT_FALSE(rShrink.passed);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillMetalInjectionMold, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::metal_injection_mold::Input in;
    in.material             = "titanium_grade5";
    in.binder_pct           = 42.0;
    in.sinter_shrinkage_pct = 19.0;
    in.wall_thickness_mm    = 1.5;

    auto out = skill::metal_injection_mold::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("metal_injection_mold"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("metal_injection_mold"));
    EXPECT_EQ(out.signature.pattern["process_family"].get<std::string>(),
              std::string("powder_metallurgy"));
    EXPECT_EQ(out.signature.pattern["material"].get<std::string>(),
              std::string("titanium_grade5"));
    EXPECT_NEAR(out.signature.pattern["binder_pct"].get<double>(),           42.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["sinter_shrinkage_pct"].get<double>(), 19.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_TRUE (out.signature.pattern["additive_workflow"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("mim_press_furnace"));
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillMetalInjectionMold, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::metal_injection_mold::Input in;
    in.material             = "stainless_17_4ph";
    in.binder_pct           = 38.0;
    in.sinter_shrinkage_pct = 16.0;
    in.wall_thickness_mm    = 2.5;

    auto out   = skill::metal_injection_mold::apply(*stock, in);
    auto cands = skill::metal_injection_mold::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("metal_injection_mold"));
    EXPECT_GT(cands[0].confidence, 0.99);
    EXPECT_EQ(cands[0].recovered_params["material"].get<std::string>(),
              std::string("stainless_17_4ph"));
    EXPECT_NEAR(cands[0].recovered_params["sinter_shrinkage_pct"].get<double>(),
                16.0, 1e-9);

    // Bare workpiece without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::metal_injection_mold::recognize(raw).empty());
}

// ─── 5. Cavity scale derives from sinter_shrinkage_pct ────────────────────
TEST(SkillMetalInjectionMold, PredictedCavityScaleMatchesShrinkage)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::metal_injection_mold::Input in;
    in.material             = "stainless_316";
    in.binder_pct           = 40.0;
    in.sinter_shrinkage_pct = 20.0;        // → scale = 1 / 0.80 = 1.25
    in.wall_thickness_mm    = 2.0;

    auto out = skill::metal_injection_mold::apply(*stock, in);
    const double expected = 1.0 / (1.0 - 20.0 / 100.0);
    EXPECT_NEAR(out.signature.pattern["predicted_cavity_scale"].get<double>(),
                expected, 1e-9);
    EXPECT_NEAR(out.signature.tooling.extra["predicted_cavity_scale"].get<double>(),
                expected, 1e-9);
}
