// @lat: [[process/test-strategy#skill round-trip]]
//
// visual_weld_inspect — VT NDT skill, metadata-only sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/visual_weld_inspect.hpp"

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

}  // namespace

// ─── 1. Apply: geometry unchanged ────────────────────────────────────────
TEST(SkillVisualWeldInspect, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::visual_weld_inspect::Input in;
    in.weld_id      = "W1";
    in.weld_size_mm = 6.0;
    in.accept       = true;

    auto out = skill::visual_weld_inspect::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillVisualWeldInspect, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::visual_weld_inspect::Input in;
    in.weld_id      = "";       // empty
    in.weld_size_mm = -1.0;     // non-positive
    in.accept       = true;

    auto r = skill::visual_weld_inspect::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::visual_weld_inspect::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillVisualWeldInspect, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::visual_weld_inspect::Input in;
    in.weld_id      = "WELD-V8";
    in.weld_size_mm = 8.0;
    in.accept       = false;

    auto out = skill::visual_weld_inspect::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("visual_weld_inspect"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("visual_weld_inspect"));
    EXPECT_EQ(out.signature.pattern["weld_id"].get<std::string>(),
              std::string("WELD-V8"));
    EXPECT_NEAR(out.signature.pattern["weld_size_mm"].get<double>(), 8.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["accept"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillVisualWeldInspect, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::visual_weld_inspect::Input in;
    in.weld_id      = "WELD-V2";
    in.weld_size_mm = 5.0;
    in.accept       = true;
    auto out = skill::visual_weld_inspect::apply(*stock, in);

    auto cands = skill::visual_weld_inspect::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["weld_size_mm"].get<double>(),
                5.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::visual_weld_inspect::recognize(raw).empty());
}

// ─── 5. Skill-specific: oversize fillet → info finding, not block ───────
TEST(SkillVisualWeldInspect, OversizeFilletEmitsInfo)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::visual_weld_inspect::Input in;
    in.weld_id      = "W1";
    in.weld_size_mm = 50.0;  // > 40 mm
    in.accept       = true;

    auto r = skill::visual_weld_inspect::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "oversize weld should NOT block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-VT-OVERSIZE"));
    EXPECT_NO_THROW(skill::visual_weld_inspect::apply(*stock, in));
}
