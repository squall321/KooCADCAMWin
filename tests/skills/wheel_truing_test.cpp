// @lat: [[process/test-strategy#skill round-trip]]
//
// wheel_truing skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of out-of-range wheel diameter / flange / run-out,
// metadata-only recognition, and approaching-scrap diameter advisory.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/wheel_truing.hpp"

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

skill::wheel_truing::Input goodInput()
{
    skill::wheel_truing::Input in;
    in.wheel_dia_mm     = 920.0;
    in.flange_height_mm = 28.0;
    in.run_out_mm       = 0.05;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillWheelTruing, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::wheel_truing::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("wheel_truing"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (diameter, flange, run-out) ────────────
TEST(SkillWheelTruing, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.wheel_dia_mm = 700.0;                 // < 780 mm
    auto r = skill::wheel_truing::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-WT-DIA"));
    EXPECT_THROW(skill::wheel_truing::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.flange_height_mm = 20.0;             // < 22 mm
    auto r2 = skill::wheel_truing::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-WT-FLANGE"));

    auto in3 = goodInput();
    in3.run_out_mm = 0.5;                    // > 0.3 mm
    auto r3 = skill::wheel_truing::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-WT-RUNOUT"));
}

// ─── 3. Signature records kind + dia/flange/runout ────────────────────────
TEST(SkillWheelTruing, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.wheel_dia_mm     = 860.0;
    in.flange_height_mm = 30.0;
    in.run_out_mm       = 0.10;
    auto out = skill::wheel_truing::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("wheel_truing"));
    EXPECT_NEAR(out.signature.pattern["wheel_dia_mm"].get<double>(),
                860.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["flange_height_mm"].get<double>(),
                30.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["run_out_mm"].get<double>(),
                0.10, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillWheelTruing, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::wheel_truing::apply(*stock, goodInput());

    auto cands = skill::wheel_truing::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["wheel_dia_mm"].get<double>(),
                920.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["flange_height_mm"].get<double>(),
                28.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::wheel_truing::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "wheel_truing cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: approaching scrap (dia in [780,840)) emits info only ────
TEST(SkillWheelTruing, ApproachingScrapEmitsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.wheel_dia_mm = 820.0;             // valid but near scrap
    auto r = skill::wheel_truing::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "approaching-scrap diameter should NOT block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-WT-SCRAP"));

    EXPECT_NO_THROW(skill::wheel_truing::apply(*stock, in));
}
