// @lat: [[process/test-strategy#compound square_case_octagonal_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/square_case_octagonal_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::square_case_octagonal_compound::Input goodInput() {
    skill::square_case_octagonal_compound::Input in;
    in.case_width_mm    = 42.0;
    in.case_height_mm   = 11.0;
    in.lug_count        = 8;
    in.screwdown_count  = 8;
    return in;
}
}  // namespace

// ─── 1. Apply produces valid shape (lugs add + corners + caseback cut) ──
TEST(SkillSquareCaseOctagonalCompound, ApplyProducesValidShape)
{
    auto stock = skill::createCuboidStock(42.0, 42.0, 11.0);

    auto in = goodInput();
    auto out = skill::square_case_octagonal_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(stock->shape());
    const double v2 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v2, v1 * 0.4);
    EXPECT_LT(v2, v1 * 1.5);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects bad case_width ─────────────────────────────────
TEST(SkillSquareCaseOctagonalCompound, ValidateRejectsBadCaseWidth)
{
    auto stock = skill::createCuboidStock(42.0, 42.0, 11.0);
    auto in = goodInput();
    in.case_width_mm = 25.0;   // < 30
    auto r = skill::square_case_octagonal_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CASE-DIA") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Validate rejects bad screwdown_count ────────────────────────────
TEST(SkillSquareCaseOctagonalCompound, ValidateRejectsBadScrewCount)
{
    auto stock = skill::createCuboidStock(42.0, 42.0, 11.0);
    auto in = goodInput();
    in.screwdown_count = 6;
    auto r = skill::square_case_octagonal_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SCREW-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records assembly + style + counts ─────────────────────
TEST(SkillSquareCaseOctagonalCompound, SignatureRecordsWatchAssembly)
{
    auto stock = skill::createCuboidStock(42.0, 42.0, 11.0);
    auto in = goodInput();
    auto out = skill::square_case_octagonal_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("square_case_octagonal_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_watch_assembly").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("watch_style").get<std::string>(),
              std::string("square_octagonal"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 5);
    EXPECT_EQ(out.signature.pattern.at("screwdown_count").get<int>(), 8);
}

// ─── 5. Recognize via metadata replay ───────────────────────────────────
TEST(SkillSquareCaseOctagonalCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(42.0, 42.0, 11.0);
    auto in = goodInput();
    auto out = skill::square_case_octagonal_compound::apply(*stock, in);

    auto cands = skill::square_case_octagonal_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 0.5);
    EXPECT_EQ(cands[0].recovered_params.at("lug_count").get<int>(), 8);
}
