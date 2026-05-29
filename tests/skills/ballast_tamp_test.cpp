// @lat: [[process/test-strategy#skill round-trip]]
//
// ballast_tamp skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of out-of-range lift, metadata-only recognition, and the
// high-speed line passes-count advisory.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ballast_tamp.hpp"

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

skill::ballast_tamp::Input goodInput()
{
    skill::ballast_tamp::Input in;
    in.lift_mm      = 20.0;
    in.line_class   = "main";
    in.passes_count = 2;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillBallastTamp, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::ballast_tamp::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("ballast_tamp"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (lift > 60 mm and lift <= 0) ───────────
TEST(SkillBallastTamp, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.lift_mm = 80.0;                // > 60 mm
    auto r = skill::ballast_tamp::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-BT-LIFT"));
    EXPECT_THROW(skill::ballast_tamp::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.lift_mm = 0.0;
    auto r2 = skill::ballast_tamp::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-BT-LIFT"));

    auto in3 = goodInput();
    in3.passes_count = 0;
    auto r3 = skill::ballast_tamp::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + lift/class/passes_count ──────────────────
TEST(SkillBallastTamp, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.lift_mm      = 35.0;
    in.line_class   = "high_speed";
    in.passes_count = 3;
    auto out = skill::ballast_tamp::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ballast_tamp"));
    EXPECT_NEAR(out.signature.pattern["lift_mm"].get<double>(),
                35.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["line_class"].get<std::string>(),
              std::string("high_speed"));
    EXPECT_EQ(out.signature.pattern["passes_count"].get<int>(), 3);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillBallastTamp, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::ballast_tamp::apply(*stock, goodInput());

    auto cands = skill::ballast_tamp::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["lift_mm"].get<double>(),
                20.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["line_class"].get<std::string>(),
              std::string("main"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::ballast_tamp::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "ballast_tamp cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: high_speed line with passes_count=1 → info, still passes
TEST(SkillBallastTamp, HighSpeedSinglePassEmitsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.line_class   = "high_speed";
    in.passes_count = 1;              // < 2 recommended
    auto r = skill::ballast_tamp::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "high_speed + single pass should NOT block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-BT-PASS"));

    EXPECT_NO_THROW(skill::ballast_tamp::apply(*stock, in));
}
