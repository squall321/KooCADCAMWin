// @lat: [[process/test-strategy#skill round-trip]]
//
// stone_cut skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection on non-positive cut length, metadata-only recognition,
// and abrasive_disc-on-granite blade-fit warning (non-blocking).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/stone_cut.hpp"

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

skill::stone_cut::Input goodInput()
{
    skill::stone_cut::Input in;
    in.stone_type   = "granite";
    in.cut_length_m = 2.0;
    in.blade_type   = "diamond_circular";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillStoneCut, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(2000.0, 1000.0, 30.0, "granite");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::stone_cut::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("stone_cut"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (cut_length_m = 0) ────────────────────
TEST(SkillStoneCut, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(2000.0, 1000.0, 30.0, "granite");

    auto in = goodInput();
    in.cut_length_m = 0.0;

    auto r = skill::stone_cut::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::stone_cut::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + stone/length/blade ──────────────────────
TEST(SkillStoneCut, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(3000.0, 1500.0, 30.0, "limestone");

    auto in = goodInput();
    in.stone_type   = "limestone";
    in.cut_length_m = 3.2;
    in.blade_type   = "diamond_wire";

    auto out = skill::stone_cut::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("stone_cut"));
    EXPECT_EQ(out.signature.pattern["stone_type"].get<std::string>(),
              std::string("limestone"));
    EXPECT_NEAR(out.signature.pattern["cut_length_m"].get<double>(),
                3.2, 1e-9);
    EXPECT_EQ(out.signature.pattern["blade_type"].get<std::string>(),
              std::string("diamond_wire"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("diamond_wire_saw"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillStoneCut, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(2000.0, 1000.0, 30.0, "granite");
    auto out = skill::stone_cut::apply(*stock, goodInput());

    auto cands = skill::stone_cut::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["stone_type"].get<std::string>(),
              std::string("granite"));
    EXPECT_NEAR(cands[0].recovered_params["cut_length_m"].get<double>(),
                2.0, 1e-9);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::stone_cut::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "stone_cut cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: abrasive_disc on granite triggers blade-fit warning ─────
TEST(SkillStoneCut, AbrasiveDiscOnGraniteEmitsBladeFitWarning)
{
    auto stock = skill::createCuboidStock(2000.0, 1000.0, 30.0, "granite");

    auto in = goodInput();
    in.blade_type = "abrasive_disc";

    auto r = skill::stone_cut::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "abrasive_disc on granite should warn but not block";
    EXPECT_TRUE(hasFinding(r, "DFM-STONE-BLADE-FIT"));
    EXPECT_NO_THROW(skill::stone_cut::apply(*stock, in));
}
