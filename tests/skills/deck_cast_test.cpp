// @lat: [[process/test-strategy#skill round-trip]]
//
// deck_cast skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of zero rebar density, metadata-only recognition, and the
// specific assertion that long-span decks emit an info finding without
// blocking apply().

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/deck_cast.hpp"

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

skill::deck_cast::Input goodInput()
{
    skill::deck_cast::Input in;
    in.span_m      = 30.0;
    in.area_m2     = 240.0;
    in.rebar_kg_m3 = 150.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillDeckCast, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::deck_cast::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("deck_cast"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillDeckCast, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.rebar_kg_m3 = 0.0;
    auto r = skill::deck_cast::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::deck_cast::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.span_m = -1.0;
    auto r2 = skill::deck_cast::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.area_m2 = 0.0;
    auto r3 = skill::deck_cast::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillDeckCast, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.span_m      = 60.0;
    in.area_m2     = 500.0;
    in.rebar_kg_m3 = 200.0;
    auto out = skill::deck_cast::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("deck_cast"));
    EXPECT_NEAR(out.signature.pattern["span_m"].get<double>(), 60.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["area_m2"].get<double>(), 500.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["rebar_kg_m3"].get<double>(), 200.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillDeckCast, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::deck_cast::apply(*stock, goodInput());

    auto cands = skill::deck_cast::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["span_m"].get<double>(), 30.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::deck_cast::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "deck_cast cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: long-span emits info but apply succeeds ─────────────────
TEST(SkillDeckCast, LongSpanIsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.span_m = 150.0;     // > 100
    auto r = skill::deck_cast::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "long-span deck should NOT block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-DECK-SPAN"));
    EXPECT_NO_THROW(skill::deck_cast::apply(*stock, in));
}
