// @lat: [[process/test-strategy#skill round-trip]]
//
// wind_blade_layup skill — 5-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/wind_blade_layup.hpp"

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

skill::wind_blade_layup::Input goodInput()
{
    skill::wind_blade_layup::Input in;
    in.ply_count  = 20;
    in.fiber_type = "E-glass";
    in.length_m   = 45.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillWindBladeLayup, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::wind_blade_layup::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("wind_blade_layup"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillWindBladeLayup, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.ply_count = 0;
    auto r = skill::wind_blade_layup::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::wind_blade_layup::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.length_m = 150.0;       // > 120 m
    auto r2 = skill::wind_blade_layup::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-BLADE-LEN"));
}

// ─── 3. Signature records kind + plies + fiber + length ──────────────────
TEST(SkillWindBladeLayup, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.ply_count  = 40;
    in.fiber_type = "carbon";
    in.length_m   = 80.0;

    auto out = skill::wind_blade_layup::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("wind_blade_layup"));
    EXPECT_EQ(out.signature.pattern["ply_count"].get<int>(), 40);
    EXPECT_EQ(out.signature.pattern["fiber_type"].get<std::string>(),
              std::string("carbon"));
    EXPECT_NEAR(out.signature.pattern["length_m"].get<double>(),
                80.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillWindBladeLayup, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::wind_blade_layup::apply(*stock, goodInput());

    auto cands = skill::wind_blade_layup::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["fiber_type"].get<std::string>(),
              std::string("E-glass"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::wind_blade_layup::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. SPECIFIC: carbon fiber stiffness_index > glass ───────────────────
TEST(SkillWindBladeLayup, CarbonFiberStiffnessIndexExceedsGlass)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto eGlass = goodInput();
    eGlass.fiber_type = "E-glass";
    auto eOut = skill::wind_blade_layup::apply(*stock, eGlass);

    auto carbonIn = goodInput();
    carbonIn.fiber_type = "carbon";
    auto cOut = skill::wind_blade_layup::apply(*stock, carbonIn);

    EXPECT_GT(cOut.signature.pattern["stiffness_index"].get<double>(),
              eOut.signature.pattern["stiffness_index"].get<double>())
        << "carbon must have higher stiffness_index than E-glass";
    EXPECT_EQ(eOut.signature.tooling.tool_type, std::string("vartm_mold"));
}
