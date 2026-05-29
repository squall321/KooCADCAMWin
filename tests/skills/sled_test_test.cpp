// @lat: [[process/test-strategy#skill round-trip]]
//
// sled_test skill — acceleration-pulse fixture no-op sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sled_test.hpp"

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

skill::sled_test::Input goodInput()
{
    skill::sled_test::Input in;
    in.pulse_g_peak = 30.0;
    in.duration_ms  = 30.0;
    in.impact_class = "crash";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ─────────────────────────────────
TEST(SkillSledTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::sled_test::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("sled_test"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (zero pulse_g_peak) ───────────────────
TEST(SkillSledTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.pulse_g_peak = 0.0;                 // invalid

    auto r = skill::sled_test::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::sled_test::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.duration_ms = -5.0;
    auto r2 = skill::sled_test::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. Signature records kind + pulse/duration/class ────────────────────
TEST(SkillSledTest, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.pulse_g_peak = 50.0;
    in.duration_ms  = 18.0;
    in.impact_class = "shock";

    auto out = skill::sled_test::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("sled_test"));
    EXPECT_NEAR(out.signature.pattern["pulse_g_peak"].get<double>(), 50.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["duration_ms"].get<double>(),  18.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["impact_class"].get<std::string>(),
              std::string("shock"));
    EXPECT_EQ(out.signature.pattern["pulse_shape"].get<std::string>(),
              std::string("haversine"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSledTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::sled_test::apply(*stock, goodInput());

    auto cands = skill::sled_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["pulse_g_peak"].get<double>(),
                30.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::sled_test::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "sled_test cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: longer pulse + higher G ⇒ larger Δv ────────────────────
TEST(SkillSledTest, LongerPulseHigherGYieldsLargerDeltaV)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto small = goodInput();
    small.pulse_g_peak = 20.0;
    small.duration_ms  = 10.0;
    auto smallOut = skill::sled_test::apply(*stock, small);

    auto big = goodInput();
    big.pulse_g_peak = 60.0;
    big.duration_ms  = 50.0;
    auto bigOut = skill::sled_test::apply(*stock, big);

    EXPECT_GT(bigOut.signature.pattern["delta_v_m_s"].get<double>(),
              smallOut.signature.pattern["delta_v_m_s"].get<double>())
        << "higher G × longer pulse should give larger Δv";
}
