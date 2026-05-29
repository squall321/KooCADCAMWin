// @lat: [[process/test-strategy#skill round-trip]]
//
// drop_test skill — destructive impact-test no-op sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drop_test.hpp"

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

skill::drop_test::Input goodInput()
{
    skill::drop_test::Input in;
    in.drop_height_mm = 1000.0;
    in.impact_axis    = "-Z";
    in.surface_type   = "concrete";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ─────────────────────────────────
TEST(SkillDropTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::drop_test::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("drop_test"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (negative drop_height) ────────────────
TEST(SkillDropTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.drop_height_mm = -10.0;            // invalid

    auto r = skill::drop_test::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::drop_test::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + drop_height/axis/surface ────────────────
TEST(SkillDropTest, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.drop_height_mm = 1500.0;
    in.impact_axis    = "+X";
    in.surface_type   = "steel";

    auto out = skill::drop_test::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("drop_test"));
    EXPECT_NEAR(out.signature.pattern["drop_height_mm"].get<double>(), 1500.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["impact_axis"].get<std::string>(),
              std::string("+X"));
    EXPECT_EQ(out.signature.pattern["surface_type"].get<std::string>(),
              std::string("steel"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_GT(out.signature.pattern["expected_g_peak"].get<double>(), 0.0);
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillDropTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::drop_test::apply(*stock, goodInput());

    auto cands = skill::drop_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["drop_height_mm"].get<double>(),
                1000.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::drop_test::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "drop_test cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: softer surface ⇒ lower estimated peak G ────────────────
TEST(SkillDropTest, SofterSurfaceLowersExpectedPeakG)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto hardIn = goodInput();
    hardIn.surface_type = "steel";
    auto hardOut = skill::drop_test::apply(*stock, hardIn);

    auto softIn = goodInput();
    softIn.surface_type = "rubber";
    auto softOut = skill::drop_test::apply(*stock, softIn);

    EXPECT_LT(softOut.signature.pattern["expected_g_peak"].get<double>(),
              hardOut.signature.pattern["expected_g_peak"].get<double>())
        << "rubber should give lower peak G than steel";
}
