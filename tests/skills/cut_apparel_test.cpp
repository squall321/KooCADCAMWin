// @lat: [[process/test-strategy#skill round-trip]]
//
// cut_apparel skill — 5-case sweep:
//   1. Apply clones geometry unchanged
//   2. Validate rejects empty pattern_id & zero layers
//   3. Signature records kind + pattern_id + cut_layers + cut_method
//   4. Recognize metadata replay
//   5. SPECIFIC: laser cut_method selects co2_laser_cutter tooling

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cut_apparel.hpp"

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

skill::cut_apparel::Input goodInput()
{
    skill::cut_apparel::Input in;
    in.pattern_id = "PT-SHIRT-A";
    in.cut_layers = 40;
    in.cut_method = "laser";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillCutApparel, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::cut_apparel::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("cut_apparel"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillCutApparel, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.pattern_id = "";    // invalid
    auto r = skill::cut_apparel::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::cut_apparel::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.cut_layers = 0;    // invalid
    auto r2 = skill::cut_apparel::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillCutApparel, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.pattern_id = "PT-JACKET-B";
    in.cut_layers = 80;
    in.cut_method = "die";

    auto out = skill::cut_apparel::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cut_apparel"));
    EXPECT_EQ(out.signature.pattern["pattern_id"].get<std::string>(),
              std::string("PT-JACKET-B"));
    EXPECT_EQ(out.signature.pattern["cut_layers"].get<int>(), 80);
    EXPECT_EQ(out.signature.pattern["cut_method"].get<std::string>(),
              std::string("die"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillCutApparel, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::cut_apparel::apply(*stock, goodInput());

    auto cands = skill::cut_apparel::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["pattern_id"].get<std::string>(),
              std::string("PT-SHIRT-A"));

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::cut_apparel::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "cut_apparel cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: cut_method dispatches tooling type correctly ────────────
TEST(SkillCutApparel, CutMethodSelectsToolingType)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto laserIn = goodInput();
    laserIn.cut_method = "laser";
    auto laserOut = skill::cut_apparel::apply(*stock, laserIn);
    EXPECT_EQ(laserOut.signature.tooling.tool_type,
              std::string("co2_laser_cutter"));

    auto dieIn = goodInput();
    dieIn.cut_method = "die";
    auto dieOut = skill::cut_apparel::apply(*stock, dieIn);
    EXPECT_EQ(dieOut.signature.tooling.tool_type,
              std::string("steel_rule_die_press"));

    auto manualIn = goodInput();
    manualIn.cut_method = "manual";
    auto manualOut = skill::cut_apparel::apply(*stock, manualIn);
    EXPECT_EQ(manualOut.signature.tooling.tool_type,
              std::string("manual_shears"));

    // Die press cycle is the fastest for the same layer count.
    EXPECT_LT(dieOut.signature.tooling.est_cycle_time_s,
              laserOut.signature.tooling.est_cycle_time_s);
    EXPECT_LT(laserOut.signature.tooling.est_cycle_time_s,
              manualOut.signature.tooling.est_cycle_time_s);
}
