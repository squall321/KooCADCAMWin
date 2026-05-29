// @lat: [[process/test-strategy#skill round-trip]]
//
// tear_test skill — trouser-leg tear-test no-op sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tear_test.hpp"

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

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

skill::tear_test::Input goodInput()
{
    skill::tear_test::Input in;
    in.tear_force_n        = 50.0;
    in.sample_thickness_mm = 2.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ─────────────────────────────────
TEST(SkillTearTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::tear_test::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("tear_test"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (zero thickness) ──────────────────────
TEST(SkillTearTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.sample_thickness_mm = 0.0;          // invalid

    auto r = skill::tear_test::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::tear_test::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.tear_force_n = -1.0;
    auto r2 = skill::tear_test::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. Signature records kind + force/thickness ─────────────────────────
TEST(SkillTearTest, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.tear_force_n        = 80.0;
    in.sample_thickness_mm = 4.0;

    auto out = skill::tear_test::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("tear_test"));
    EXPECT_NEAR(out.signature.pattern["tear_force_n"].get<double>(),         80.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["sample_thickness_mm"].get<double>(),   4.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["tear_strength_n_mm"].get<double>(),   20.0, 1e-6);
    EXPECT_EQ(out.signature.pattern["specimen_geometry"].get<std::string>(),
              std::string("trouser_leg"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillTearTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::tear_test::apply(*stock, goodInput());

    auto cands = skill::tear_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["tear_force_n"].get<double>(),
                50.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::tear_test::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "tear_test cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: tear_strength = tear_force / thickness ─────────────────
TEST(SkillTearTest, TearStrengthEqualsForceOverThickness)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.tear_force_n        = 100.0;
    in.sample_thickness_mm = 2.5;

    auto out = skill::tear_test::apply(*stock, in);
    const double S = out.signature.pattern["tear_strength_n_mm"].get<double>();
    EXPECT_NEAR(S, 100.0 / 2.5, 1e-6);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("tensile_tear_grips"));
}
