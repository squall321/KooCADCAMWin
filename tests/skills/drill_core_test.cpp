// @lat: [[process/test-strategy#skill round-trip]]
//
// drill_core — mining exploration core drilling.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. DiamondMethodSelectsDiamondBit

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_core.hpp"

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

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillDrillCore, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 200.0);
    const double volBefore = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::drill_core::Input in;
    in.hole_dia_mm  = 96.0;
    in.depth_m      = 250.0;
    in.drill_method = "diamond";

    auto out = skill::drill_core::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("drill_core"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillDrillCore, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);

    skill::drill_core::Input badDia;
    badDia.hole_dia_mm  = 0.0;
    badDia.depth_m      = 100.0;
    badDia.drill_method = "diamond";
    {
        auto r = skill::drill_core::validate(*stock, badDia);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::drill_core::apply(*stock, badDia), skill::SkillError);

    skill::drill_core::Input badMethod;
    badMethod.hole_dia_mm  = 76.0;
    badMethod.depth_m      = 100.0;
    badMethod.drill_method = "laser";
    {
        auto r = skill::drill_core::validate(*stock, badMethod);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-DRILL-METHOD"));
    }
    EXPECT_THROW(skill::drill_core::apply(*stock, badMethod), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillDrillCore, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);

    skill::drill_core::Input in;
    in.hole_dia_mm  = 76.0;
    in.depth_m      = 150.0;
    in.drill_method = "rotary";

    auto out = skill::drill_core::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("drill_core"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("drill_core"));
    EXPECT_NEAR(out.signature.pattern["hole_dia_mm"].get<double>(),
                76.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["depth_m"].get<double>(),
                150.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["drill_method"].get<std::string>(),
              std::string("rotary"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillDrillCore, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);

    skill::drill_core::Input in;
    in.hole_dia_mm  = 60.0;
    in.depth_m      = 80.0;
    in.drill_method = "percussive";

    auto out   = skill::drill_core::apply(*stock, in);
    auto cands = skill::drill_core::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["drill_method"].get<std::string>(),
              std::string("percussive"));
    EXPECT_NEAR(cands[0].recovered_params["depth_m"].get<double>(),
                80.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::drill_core::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: diamond method selects diamond_core_bit tooling ─────────
TEST(SkillDrillCore, DiamondMethodSelectsDiamondBit)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);

    skill::drill_core::Input in;
    in.hole_dia_mm  = 76.0;
    in.depth_m      = 100.0;
    in.drill_method = "diamond";

    auto out = skill::drill_core::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("diamond_core_bit"));
    EXPECT_EQ(out.signature.tooling.tool_material,
              std::string("impregnated_diamond"));
}
