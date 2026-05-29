// @lat: [[process/test-strategy#skill round-trip]]
//
// regrind_polymer skill — 5-case sweep covering:
//   1. identity geometry on apply
//   2. DFM rejects bad input (non-positive grind size; out-of-range
//      contamination; empty polymer_type)
//   3. signature carries skill kind + polymer_type + grind_size_mm +
//      contamination_pct
//   4. recognize via metadata replay
//   5. specific: high contamination → warning; unknown polymer → info; proceeds

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/regrind_polymer.hpp"

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

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillRegrindPolymer, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::regrind_polymer::Input in;
    in.polymer_type      = "PET";
    in.grind_size_mm     = 8.0;
    in.contamination_pct = 0.5;

    auto out = skill::regrind_polymer::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("regrind_polymer"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillRegrindPolymer, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::regrind_polymer::Input badSize;
    badSize.polymer_type      = "PET";
    badSize.grind_size_mm     = 0.0;        // invalid
    badSize.contamination_pct = 1.0;
    auto r1 = skill::regrind_polymer::validate(*stock, badSize);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::regrind_polymer::apply(*stock, badSize), skill::SkillError);

    skill::regrind_polymer::Input badContHi;
    badContHi.polymer_type      = "PET";
    badContHi.grind_size_mm     = 5.0;
    badContHi.contamination_pct = 100.0;    // = 100 → invalid (must be < 100)
    auto r2 = skill::regrind_polymer::validate(*stock, badContHi);
    EXPECT_FALSE(r2.passed);

    skill::regrind_polymer::Input badContNeg;
    badContNeg.polymer_type      = "PET";
    badContNeg.grind_size_mm     = 5.0;
    badContNeg.contamination_pct = -1.0;    // invalid
    auto r3 = skill::regrind_polymer::validate(*stock, badContNeg);
    EXPECT_FALSE(r3.passed);

    skill::regrind_polymer::Input emptyType;
    emptyType.polymer_type      = "";       // invalid
    emptyType.grind_size_mm     = 5.0;
    emptyType.contamination_pct = 1.0;
    auto r4 = skill::regrind_polymer::validate(*stock, emptyType);
    EXPECT_FALSE(r4.passed);
}

// ─── 3. Signature records skill kind + key params ────────────────────────
TEST(SkillRegrindPolymer, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::regrind_polymer::Input in;
    in.polymer_type      = "HDPE";
    in.grind_size_mm     = 10.0;
    in.contamination_pct = 2.0;

    auto out = skill::regrind_polymer::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("regrind_polymer"));
    EXPECT_EQ(out.signature.pattern["polymer_type"].get<std::string>(),
              std::string("HDPE"));
    EXPECT_NEAR(out.signature.pattern["grind_size_mm"].get<double>(),
                10.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["contamination_pct"].get<double>(),
                2.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("granulator"));
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillRegrindPolymer, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::regrind_polymer::Input in;
    in.polymer_type      = "ABS";
    in.grind_size_mm     = 4.0;
    in.contamination_pct = 1.5;

    auto out = skill::regrind_polymer::apply(*stock, in);

    auto cands = skill::regrind_polymer::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["polymer_type"].get<std::string>(),
              std::string("ABS"));

    skill::Workpiece raw(out.workpiece->shape());
    auto empty = skill::regrind_polymer::recognize(raw);
    EXPECT_TRUE(empty.empty())
        << "regrind_polymer cannot be recognized geometrically";
}

// ─── 5. Specific: high contamination warning + unknown polymer info ──────
TEST(SkillRegrindPolymer, HighContaminationWarningAndUnknownPolymerInfo)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::regrind_polymer::Input in;
    in.polymer_type      = "PEEK_blend";       // unrecognised → info
    in.grind_size_mm     = 6.0;
    in.contamination_pct = 9.5;                // > 5 → warning

    auto r = skill::regrind_polymer::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "warnings/info must not block apply";
    EXPECT_TRUE(hasFinding(r, "DFM-REGRIND-CONT"));
    EXPECT_TRUE(hasFinding(r, "DFM-REGRIND-TYPE"));
    EXPECT_NO_THROW(skill::regrind_polymer::apply(*stock, in));
}
