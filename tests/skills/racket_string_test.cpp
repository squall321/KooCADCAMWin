// @lat: [[process/test-strategy#skill round-trip]]
//
// racket_string skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of out-of-range tension, metadata-only recognition, and a
// skill-specific assertion that the "16x19" pattern parses into mains+crosses.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/racket_string.hpp"

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

skill::racket_string::Input goodInput()
{
    skill::racket_string::Input in;
    in.tension_lbf    = 55.0;
    in.string_pattern = "16x19";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillRacketString, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::racket_string::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("racket_string"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ───────────────────────────────────────
TEST(SkillRacketString, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    // Out-of-range tension.
    auto in = goodInput();
    in.tension_lbf = 95.0;       // > 80 lbf limit
    auto r = skill::racket_string::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-RACKET-TENSION"));
    EXPECT_THROW(skill::racket_string::apply(*stock, in), skill::SkillError);

    // Malformed pattern.
    auto in2 = goodInput();
    in2.string_pattern = "weird";
    auto r2 = skill::racket_string::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-RACKET-PATTERN"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillRacketString, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.tension_lbf    = 62.0;
    in.string_pattern = "18x20";

    auto out = skill::racket_string::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("racket_string"));
    EXPECT_NEAR(out.signature.pattern["tension_lbf"].get<double>(), 62.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["string_pattern"].get<std::string>(),
              std::string("18x20"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillRacketString, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::racket_string::apply(*stock, goodInput());

    auto cands = skill::racket_string::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["tension_lbf"].get<double>(),
                55.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["string_pattern"].get<std::string>(),
              std::string("16x19"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::racket_string::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "racket_string cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: pattern "16x19" parses into 16 mains + 19 crosses ──────
TEST(SkillRacketString, PatternParsedIntoMainAndCrossCounts)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::racket_string::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.pattern["main_count"].get<int>(),  16);
    EXPECT_EQ(out.signature.pattern["cross_count"].get<int>(), 19);
    EXPECT_EQ(out.signature.pattern["total_strings"].get<int>(),
              16 + 19);
}
