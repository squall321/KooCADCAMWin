// @lat: [[process/test-strategy#skill round-trip]]
//
// hot_isostatic_press — HIP densification (geometric no-op).
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndKeyParams
//   4. RecognizeMetadataReplay
//   5. Expected density grows with cycle severity

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hot_isostatic_press.hpp"

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

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillHotIsostaticPress, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0, "titanium_grade5");
    const double volBefore   = volumeOf(stock->shape());
    const int    facesBefore = stock->faceCount();
    const int    edgesBefore = stock->edgeCount();

    skill::hot_isostatic_press::Input in;
    in.material     = "titanium_grade5";
    in.temp_c       = 950.0;
    in.pressure_mpa = 100.0;
    in.hold_min     = 120.0;

    auto out = skill::hot_isostatic_press::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), facesBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgesBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillHotIsostaticPress, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);

    skill::hot_isostatic_press::Input zeroT;
    zeroT.material     = "stainless_316";
    zeroT.temp_c       = 0.0;        // invalid
    zeroT.pressure_mpa = 150.0;
    zeroT.hold_min     = 120.0;
    auto r1 = skill::hot_isostatic_press::validate(*stock, zeroT);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::hot_isostatic_press::apply(*stock, zeroT),
                 skill::SkillError);

    skill::hot_isostatic_press::Input zeroP;
    zeroP.material     = "stainless_316";
    zeroP.temp_c       = 1150.0;
    zeroP.pressure_mpa = 0.0;        // invalid
    zeroP.hold_min     = 120.0;
    auto r2 = skill::hot_isostatic_press::validate(*stock, zeroP);
    EXPECT_FALSE(r2.passed);

    skill::hot_isostatic_press::Input zeroHold;
    zeroHold.material     = "stainless_316";
    zeroHold.temp_c       = 1150.0;
    zeroHold.pressure_mpa = 150.0;
    zeroHold.hold_min     = 0.0;     // invalid
    auto r3 = skill::hot_isostatic_press::validate(*stock, zeroHold);
    EXPECT_FALSE(r3.passed);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillHotIsostaticPress, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);

    skill::hot_isostatic_press::Input in;
    in.material     = "inconel_718";
    in.temp_c       = 1175.0;
    in.pressure_mpa = 172.0;
    in.hold_min     = 240.0;

    auto out = skill::hot_isostatic_press::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("hot_isostatic_press"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("hot_isostatic_press"));
    EXPECT_EQ(out.signature.pattern["process_family"].get<std::string>(),
              std::string("powder_metallurgy"));
    EXPECT_NEAR(out.signature.pattern["temp_c"].get<double>(),       1175.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["pressure_mpa"].get<double>(),  172.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["hold_min"].get<double>(),      240.0, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("hip_vessel"));
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillHotIsostaticPress, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);

    skill::hot_isostatic_press::Input in;
    in.material     = "stainless_316";
    in.temp_c       = 1150.0;
    in.pressure_mpa = 150.0;
    in.hold_min     = 180.0;

    auto out   = skill::hot_isostatic_press::apply(*stock, in);
    auto cands = skill::hot_isostatic_press::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("hot_isostatic_press"));
    EXPECT_GT(cands[0].confidence, 0.99);
    EXPECT_NEAR(cands[0].recovered_params["pressure_mpa"].get<double>(),
                150.0, 1e-9);

    // Stripped of metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::hot_isostatic_press::recognize(raw).empty());
}

// ─── 5. Cycle severity drives expected density ───────────────────────────
TEST(SkillHotIsostaticPress, DensityIncreasesWithCycleSeverity)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);

    skill::hot_isostatic_press::Input mild;
    mild.material     = "stainless_316";
    mild.temp_c       = 950.0;
    mild.pressure_mpa = 80.0;
    mild.hold_min     = 60.0;

    skill::hot_isostatic_press::Input aggressive;
    aggressive.material     = "stainless_316";
    aggressive.temp_c       = 1200.0;
    aggressive.pressure_mpa = 200.0;
    aggressive.hold_min     = 480.0;

    auto outMild  = skill::hot_isostatic_press::apply(*stock, mild);
    auto outAggr  = skill::hot_isostatic_press::apply(*stock, aggressive);

    const double dMild = outMild.signature.pattern["expected_density_pct"].get<double>();
    const double dAggr = outAggr.signature.pattern["expected_density_pct"].get<double>();

    EXPECT_GT(dAggr, dMild);
    EXPECT_LE(dAggr, 100.0);
    EXPECT_GE(dMild, 99.0);   // by construction starts at 99 %
}
