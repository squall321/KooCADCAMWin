// @lat: [[process/test-strategy#compound gmt_chronograph_case_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gmt_chronograph_case_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::gmt_chronograph_case_compound::Input goodInput() {
    skill::gmt_chronograph_case_compound::Input in;
    in.case_dia_mm    = 42.0;
    in.case_height_mm = 14.0;
    in.subdial_count  = 3;
    in.pusher_count   = 2;
    return in;
}
}  // namespace

// ─── 1. Apply removes material (subdial + pusher + bezel + caseback) ────
TEST(SkillGMTChronographCaseCompound, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(42.0, 14.0);

    auto in = goodInput();
    auto out = skill::gmt_chronograph_case_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(stock->shape());
    const double v2 = volumeOf(out.workpiece->shape());
    EXPECT_LT(v2, v1);
    EXPECT_GT(v2, v1 * 0.5);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects bad case_dia ───────────────────────────────────
TEST(SkillGMTChronographCaseCompound, ValidateRejectsBadCaseDia)
{
    auto stock = skill::createCylindricalStock(42.0, 14.0);
    auto in = goodInput();
    in.case_dia_mm = 30.0;   // < 38
    auto r = skill::gmt_chronograph_case_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CASE-DIA") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Validate rejects bad subdial_count ──────────────────────────────
TEST(SkillGMTChronographCaseCompound, ValidateRejectsBadSubdialCount)
{
    auto stock = skill::createCylindricalStock(42.0, 14.0);
    auto in = goodInput();
    in.subdial_count = 4;
    auto r = skill::gmt_chronograph_case_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SUBDIAL") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records assembly + watch_style + count ────────────────
TEST(SkillGMTChronographCaseCompound, SignatureRecordsWatchAssembly)
{
    auto stock = skill::createCylindricalStock(42.0, 14.0);
    auto in = goodInput();
    auto out = skill::gmt_chronograph_case_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("gmt_chronograph_case_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_watch_assembly").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("watch_style").get<std::string>(),
              std::string("gmt_chronograph"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 7);
}

// ─── 5. Recognize via metadata replay ───────────────────────────────────
TEST(SkillGMTChronographCaseCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(42.0, 14.0);
    auto in = goodInput();
    auto out = skill::gmt_chronograph_case_compound::apply(*stock, in);

    auto cands = skill::gmt_chronograph_case_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 0.5);
    EXPECT_EQ(cands[0].recovered_params.at("subdial_count").get<int>(), 3);
}
