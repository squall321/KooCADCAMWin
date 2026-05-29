// @lat: [[process/test-strategy#skill round-trip]]
//
// xray_weld_inspect — radiographic NDT skill, metadata-only sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/xray_weld_inspect.hpp"

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
TEST(SkillXrayWeldInspect, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::xray_weld_inspect::Input in;
    in.weld_id      = "W1";
    in.kv           = 200.0;
    in.exposure_min = 3.0;
    in.defect_class = "D1";

    auto out = skill::xray_weld_inspect::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillXrayWeldInspect, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::xray_weld_inspect::Input in;
    in.weld_id      = "";          // empty
    in.kv           = 800.0;       // out of range
    in.exposure_min = -1.0;        // non-positive
    in.defect_class = "X9";        // not allowed

    auto r = skill::xray_weld_inspect::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::xray_weld_inspect::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillXrayWeldInspect, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::xray_weld_inspect::Input in;
    in.weld_id      = "WELD-A7";
    in.kv           = 180.0;
    in.exposure_min = 2.5;
    in.defect_class = "D2";

    auto out = skill::xray_weld_inspect::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("xray_weld_inspect"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("xray_weld_inspect"));
    EXPECT_EQ(out.signature.pattern["weld_id"].get<std::string>(),
              std::string("WELD-A7"));
    EXPECT_NEAR(out.signature.pattern["kv"].get<double>(),           180.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["exposure_min"].get<double>(),   2.5, 1e-9);
    EXPECT_EQ(out.signature.pattern["defect_class"].get<std::string>(),
              std::string("D2"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillXrayWeldInspect, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::xray_weld_inspect::Input in;
    in.weld_id      = "WELD-B3";
    in.kv           = 220.0;
    in.exposure_min = 4.0;
    in.defect_class = "D1";
    auto out = skill::xray_weld_inspect::apply(*stock, in);

    auto cands = skill::xray_weld_inspect::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["weld_id"].get<std::string>(),
              std::string("WELD-B3"));
    EXPECT_EQ(cands[0].recovered_params["defect_class"].get<std::string>(),
              std::string("D1"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::xray_weld_inspect::recognize(raw).empty());
}

// ─── 5. Skill-specific: AWS D1 code reference + tool_type ────────────────
TEST(SkillXrayWeldInspect, ToolingReflectsRadiographyAndCode)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::xray_weld_inspect::Input in;
    in.weld_id      = "W1";
    in.kv           = 200.0;
    in.exposure_min = 5.0;
    in.defect_class = "D3";

    auto out = skill::xray_weld_inspect::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("xray_radiograph"));
    EXPECT_EQ(out.signature.pattern["code_reference"].get<std::string>(),
              std::string("AWS_D1"));
    // exposure time → est_cycle_time_s
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 5.0 * 60.0, 1e-9);
}
