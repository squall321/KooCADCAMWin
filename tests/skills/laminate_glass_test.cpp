// @lat: [[process/test-strategy#skill round-trip]]
//
// laminate_glass skill — metadata-only PVB laminate recording.  Covers:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input (autoclave_temp_c above PVB window)
//   3. signature records kind + key params
//   4. recognize via metadata replay
//   5. interlayer_material is "PVB" and tool_type is "autoclave"

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/laminate_glass.hpp"

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
TEST(SkillLaminateGlass, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(300.0, 200.0, 12.76, "laminate_stack");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::laminate_glass::Input in;
    in.interlayer_thickness_mm = 0.76;
    in.autoclave_temp_c        = 140.0;
    in.autoclave_pressure_kpa  = 1200.0;

    auto out = skill::laminate_glass::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("laminate_glass"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects bad input (autoclave_temp_c above 160 °C) ────────
TEST(SkillLaminateGlass, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 8.76, "laminate_stack");

    skill::laminate_glass::Input in;
    in.interlayer_thickness_mm = 0.76;
    in.autoclave_temp_c        = 200.0;    // > 160 → DFM-LAMI-TEMP error
    in.autoclave_pressure_kpa  = 1200.0;

    auto r = skill::laminate_glass::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LAMI-TEMP") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::laminate_glass::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillLaminateGlass, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(250.0, 180.0, 12.0, "laminate_stack");

    skill::laminate_glass::Input in;
    in.interlayer_thickness_mm = 1.52;
    in.autoclave_temp_c        = 145.0;
    in.autoclave_pressure_kpa  = 1300.0;

    auto out = skill::laminate_glass::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("laminate_glass"));
    EXPECT_NEAR(out.signature.pattern["interlayer_thickness_mm"].get<double>(),
                1.52, 1e-9);
    EXPECT_NEAR(out.signature.pattern["autoclave_temp_c"].get<double>(),
                145.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["autoclave_pressure_kpa"].get<double>(),
                1300.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillLaminateGlass, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(150.0, 100.0, 7.0, "laminate_stack");

    skill::laminate_glass::Input in;
    in.interlayer_thickness_mm = 0.38;
    in.autoclave_temp_c        = 135.0;
    in.autoclave_pressure_kpa  = 1100.0;

    auto out = skill::laminate_glass::apply(*stock, in);
    auto cands = skill::laminate_glass::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["interlayer_thickness_mm"].get<double>(),
                0.38, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::laminate_glass::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: interlayer_material is "PVB" and tool_type is autoclave ─
TEST(SkillLaminateGlass, PvbInterlayerAndAutoclaveTool)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 8.0, "laminate_stack");

    skill::laminate_glass::Input in;
    in.interlayer_thickness_mm = 0.76;
    in.autoclave_temp_c        = 140.0;
    in.autoclave_pressure_kpa  = 1200.0;

    auto out = skill::laminate_glass::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["interlayer_material"].get<std::string>(),
              std::string("PVB"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("autoclave"));
    EXPECT_EQ(out.signature.tooling.tool_material,
              std::string("steel_pressure_vessel"));
}
