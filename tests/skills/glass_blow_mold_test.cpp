// @lat: [[process/test-strategy#skill round-trip]]
//
// glass_blow_mold skill — metadata-only blown-container forming.  Covers:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input (blow_pressure_kpa out of range)
//   3. signature records kind + key params
//   4. recognize via metadata replay
//   5. tool_type is "blow_mold" and cast_iron tool_material is recorded

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/glass_blow_mold.hpp"

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
TEST(SkillGlassBlowMold, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(65.0, 200.0, "soda_lime_glass");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::glass_blow_mold::Input in;
    in.mold_id           = "BM-500ml-v3";
    in.parison_weight_g  = 320.0;
    in.blow_pressure_kpa = 250.0;

    auto out = skill::glass_blow_mold::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("glass_blow_mold"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects bad input (blow_pressure_kpa > 500) ──────────────
TEST(SkillGlassBlowMold, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(50.0, 100.0, "soda_lime_glass");

    skill::glass_blow_mold::Input in;
    in.mold_id           = "BM-jar-v1";
    in.parison_weight_g  = 150.0;
    in.blow_pressure_kpa = 900.0;     // > 500 → DFM-BLOW-PRESSURE error

    auto r = skill::glass_blow_mold::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BLOW-PRESSURE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::glass_blow_mold::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillGlassBlowMold, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(40.0, 80.0, "soda_lime_glass");

    skill::glass_blow_mold::Input in;
    in.mold_id           = "BM-750ml-wine";
    in.parison_weight_g  = 500.0;
    in.blow_pressure_kpa = 300.0;

    auto out = skill::glass_blow_mold::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("glass_blow_mold"));
    EXPECT_EQ(out.signature.pattern["mold_id"].get<std::string>(),
              std::string("BM-750ml-wine"));
    EXPECT_NEAR(out.signature.pattern["parison_weight_g"].get<double>(),  500.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["blow_pressure_kpa"].get<double>(), 300.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("blow_mold"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillGlassBlowMold, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(45.0, 90.0, "soda_lime_glass");

    skill::glass_blow_mold::Input in;
    in.mold_id           = "BM-can-v2";
    in.parison_weight_g  = 180.0;
    in.blow_pressure_kpa = 200.0;

    auto out = skill::glass_blow_mold::apply(*stock, in);
    auto cands = skill::glass_blow_mold::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["mold_id"].get<std::string>(),
              std::string("BM-can-v2"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::glass_blow_mold::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: cast_iron tooling material recorded ─────────────────────
TEST(SkillGlassBlowMold, ToolingMaterialIsCastIron)
{
    auto stock = skill::createCylindricalStock(50.0, 100.0, "soda_lime_glass");

    skill::glass_blow_mold::Input in;
    in.mold_id           = "BM-330ml-v1";
    in.parison_weight_g  = 220.0;
    in.blow_pressure_kpa = 200.0;

    auto out = skill::glass_blow_mold::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("cast_iron"));
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 10.0, 1e-9);
}
