// @lat: [[process/test-strategy#skill round-trip]]
//
// float_glass skill — metadata-only Pilkington flat-sheet recording.  Covers:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input (thickness below minimum)
//   3. signature records kind + key params
//   4. recognize via metadata replay
//   5. tool_type is "tin_bath" and tool_material is "molten_tin"

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/float_glass.hpp"

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

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillFloatGlass, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 4.0, "soda_lime_glass");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::float_glass::Input in;
    in.glass_thickness_mm  = 4.0;
    in.ribbon_speed_m_min  = 12.0;

    auto out = skill::float_glass::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("float_glass"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects bad input (thickness < 0.4) ──────────────────────
TEST(SkillFloatGlass, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 1.0, "soda_lime_glass");

    skill::float_glass::Input in;
    in.glass_thickness_mm  = 0.2;     // < 0.4 → DFM-FLOAT-THICK error
    in.ribbon_speed_m_min  = 10.0;

    auto r = skill::float_glass::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FLOAT-THICK") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::float_glass::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillFloatGlass, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 6.0, "soda_lime_glass");

    skill::float_glass::Input in;
    in.glass_thickness_mm  = 6.0;
    in.ribbon_speed_m_min  = 9.5;

    auto out = skill::float_glass::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("float_glass"));
    EXPECT_NEAR(out.signature.pattern["glass_thickness_mm"].get<double>(),  6.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["ribbon_speed_m_min"].get<double>(), 9.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("tin_bath"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFloatGlass, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 3.0, "soda_lime_glass");

    skill::float_glass::Input in;
    in.glass_thickness_mm  = 3.0;
    in.ribbon_speed_m_min  = 15.0;

    auto out = skill::float_glass::apply(*stock, in);
    auto cands = skill::float_glass::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["glass_thickness_mm"].get<double>(),
                3.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::float_glass::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: tin bath is recorded as the working medium ──────────────
TEST(SkillFloatGlass, TinBathIsRecordedAsToolingMaterial)
{
    auto stock = skill::createCuboidStock(250.0, 250.0, 5.0, "soda_lime_glass");

    skill::float_glass::Input in;
    in.glass_thickness_mm  = 5.0;
    in.ribbon_speed_m_min  = 11.0;

    auto out = skill::float_glass::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("molten_tin"));
    EXPECT_EQ(out.signature.tooling.tool_type,     std::string("tin_bath"));
}
