// @lat: [[process/test-strategy#skill round-trip]]
//
// transformer_wind skill — 5-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/transformer_wind.hpp"

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

skill::transformer_wind::Input goodInput()
{
    skill::transformer_wind::Input in;
    in.primary_turns   = 200;
    in.secondary_turns = 20;
    in.wire_dia_mm     = 1.5;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillTransformerWind, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::transformer_wind::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("transformer_wind"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillTransformerWind, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.primary_turns   = 0;
    in.secondary_turns = -5;

    auto r = skill::transformer_wind::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::transformer_wind::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.wire_dia_mm = 12.0;        // out of range
    auto r2 = skill::transformer_wind::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-XFMR-WIRE"));
}

// ─── 3. Signature records kind + turns + wire + ratio ────────────────────
TEST(SkillTransformerWind, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.primary_turns   = 1000;
    in.secondary_turns = 50;
    in.wire_dia_mm     = 0.5;

    auto out = skill::transformer_wind::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("transformer_wind"));
    EXPECT_EQ(out.signature.pattern["primary_turns"].get<int>(), 1000);
    EXPECT_EQ(out.signature.pattern["secondary_turns"].get<int>(), 50);
    EXPECT_NEAR(out.signature.pattern["wire_dia_mm"].get<double>(),
                0.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["turns_ratio"].get<double>(),
                20.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillTransformerWind, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::transformer_wind::apply(*stock, goodInput());

    auto cands = skill::transformer_wind::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["primary_turns"].get<int>(), 200);
    EXPECT_EQ(cands[0].recovered_params["secondary_turns"].get<int>(), 20);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::transformer_wind::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. SPECIFIC: turns_ratio == N₁/N₂ exact ─────────────────────────────
TEST(SkillTransformerWind, TurnsRatioMatchesN1OverN2)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::transformer_wind::Input in;
    in.primary_turns   = 240;
    in.secondary_turns = 24;
    in.wire_dia_mm     = 2.0;

    auto out = skill::transformer_wind::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["turns_ratio"].get<double>(),
                10.0, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("transformer_winder"));
}
