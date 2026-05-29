// @lat: [[process/test-strategy#skill round-trip]]
//
// ultrasonic_weld_inspect — UT NDT skill, metadata-only sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ultrasonic_weld_inspect.hpp"

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

// ─── 1. Apply: geometry unchanged ────────────────────────────────────────
TEST(SkillUltrasonicWeldInspect, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::ultrasonic_weld_inspect::Input in;
    in.weld_id      = "W1";
    in.freq_mhz     = 5.0;
    in.defect_class = "D1";

    auto out = skill::ultrasonic_weld_inspect::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillUltrasonicWeldInspect, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::ultrasonic_weld_inspect::Input in;
    in.weld_id      = "";
    in.freq_mhz     = 100.0;       // > 25
    in.defect_class = "C5";        // not allowed

    auto r = skill::ultrasonic_weld_inspect::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::ultrasonic_weld_inspect::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillUltrasonicWeldInspect, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::ultrasonic_weld_inspect::Input in;
    in.weld_id      = "WELD-U2";
    in.freq_mhz     = 7.5;
    in.defect_class = "D2";

    auto out = skill::ultrasonic_weld_inspect::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("ultrasonic_weld_inspect"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ultrasonic_weld_inspect"));
    EXPECT_EQ(out.signature.pattern["weld_id"].get<std::string>(),
              std::string("WELD-U2"));
    EXPECT_NEAR(out.signature.pattern["freq_mhz"].get<double>(), 7.5, 1e-9);
    EXPECT_EQ(out.signature.pattern["defect_class"].get<std::string>(),
              std::string("D2"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillUltrasonicWeldInspect, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::ultrasonic_weld_inspect::Input in;
    in.weld_id      = "WELD-U7";
    in.freq_mhz     = 10.0;
    in.defect_class = "D3";
    auto out = skill::ultrasonic_weld_inspect::apply(*stock, in);

    auto cands = skill::ultrasonic_weld_inspect::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["weld_id"].get<std::string>(),
              std::string("WELD-U7"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::ultrasonic_weld_inspect::recognize(raw).empty());
}

// ─── 5. Skill-specific: phased-array probe + AWS_D1 ref ──────────────────
TEST(SkillUltrasonicWeldInspect, ToolingReflectsProbeAndCode)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::ultrasonic_weld_inspect::Input in;
    in.weld_id      = "W1";
    in.freq_mhz     = 5.0;
    in.defect_class = "D1";

    auto out = skill::ultrasonic_weld_inspect::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("ut_phased_array_probe"));
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("piezo_pzt"));
    EXPECT_EQ(out.signature.pattern["code_reference"].get<std::string>(),
              std::string("AWS_D1"));
}
