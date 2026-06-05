// @lat: [[process/test-strategy#compound smart_watch_aluminum_band_lug_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/smart_watch_aluminum_band_lug_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::smart_watch_aluminum_band_lug_compound::Input goodInput() {
    skill::smart_watch_aluminum_band_lug_compound::Input in;
    in.case_w_mm     = 41.0;
    in.case_h_mm     = 35.0;
    in.case_d_mm     = 10.0;
    in.band_lug_type = "spring_bar";
    in.band_width_mm = 22.0;
    return in;
}
}  // namespace

// ─── 1. Apply removes material (corner cuts + lugs + window + crown + grilles) ─
TEST(SkillSmartWatchAluminumBandLugCompound, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(41.0, 35.0, 10.0);

    auto in = goodInput();
    auto out = skill::smart_watch_aluminum_band_lug_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(stock->shape());
    const double v2 = volumeOf(out.workpiece->shape());
    EXPECT_LT(v2, v1);
    EXPECT_GT(v2, v1 * 0.4);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects bad case_d ─────────────────────────────────────
TEST(SkillSmartWatchAluminumBandLugCompound, ValidateRejectsBadCaseDepth)
{
    auto stock = skill::createCuboidStock(41.0, 35.0, 10.0);
    auto in = goodInput();
    in.case_d_mm = 4.0;   // < 6
    auto r = skill::smart_watch_aluminum_band_lug_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CASE-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Validate rejects bad band_lug_type ──────────────────────────────
TEST(SkillSmartWatchAluminumBandLugCompound, ValidateRejectsBadLugType)
{
    auto stock = skill::createCuboidStock(41.0, 35.0, 10.0);
    auto in = goodInput();
    in.band_lug_type = "velcro";   // unknown
    auto r = skill::smart_watch_aluminum_band_lug_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BAND-LUG") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records smart_aluminum style + lug_type ───────────────
TEST(SkillSmartWatchAluminumBandLugCompound, SignatureRecordsWatchAssembly)
{
    auto stock = skill::createCuboidStock(41.0, 35.0, 10.0);
    auto in = goodInput();
    auto out = skill::smart_watch_aluminum_band_lug_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("smart_watch_aluminum_band_lug_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_watch_assembly").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("watch_style").get<std::string>(),
              std::string("smart_aluminum"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 6);
    EXPECT_EQ(out.signature.pattern.at("band_lug_type").get<std::string>(),
              std::string("spring_bar"));
}

// ─── 5. Recognize via metadata replay ───────────────────────────────────
TEST(SkillSmartWatchAluminumBandLugCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(41.0, 35.0, 10.0);
    auto in = goodInput();
    auto out = skill::smart_watch_aluminum_band_lug_compound::apply(*stock, in);

    auto cands = skill::smart_watch_aluminum_band_lug_compound::recognize(
        *out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 0.5);
    EXPECT_EQ(cands[0].recovered_params.at("band_lug_type").get<std::string>(),
              std::string("spring_bar"));
}
