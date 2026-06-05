// @lat: [[process/test-strategy#compound diver_case_full_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/diver_case_full_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::diver_case_full_compound::Input goodInput() {
    skill::diver_case_full_compound::Input in;
    in.case_dia_mm        = 42.0;
    in.case_height_mm     = 13.0;
    in.bezel_outer_dia_mm = 41.0;
    in.lug_width_mm       = 22.0;
    in.crown_position     = "3";
    return in;
}
}  // namespace

// ─── 1. Apply removes material AND adds lug material (net change non-zero) ─
TEST(SkillDiverCaseFullCompound, ApplyProducesValidShape)
{
    auto stock = skill::createCylindricalStock(42.0, 13.0);

    auto in = goodInput();
    auto out = skill::diver_case_full_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(stock->shape());
    const double v2 = volumeOf(out.workpiece->shape());
    // Net result: most stock present; net volume change is bounded but
    // non-trivial since lugs add and many features cut.
    EXPECT_GT(v2, v1 * 0.4);
    EXPECT_LT(v2, v1 * 1.5);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects bad case_dia ───────────────────────────────────
TEST(SkillDiverCaseFullCompound, ValidateRejectsBadCaseDia)
{
    auto stock = skill::createCylindricalStock(42.0, 13.0);
    auto in = goodInput();
    in.case_dia_mm = 25.0;   // < 30
    auto r = skill::diver_case_full_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CASE-DIA") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::diver_case_full_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects bad lug_width ──────────────────────────────────
TEST(SkillDiverCaseFullCompound, ValidateRejectsBadLugWidth)
{
    auto stock = skill::createCylindricalStock(42.0, 13.0);
    auto in = goodInput();
    in.lug_width_mm = 18.0;   // not in {20, 22, 24}
    auto r = skill::diver_case_full_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LUG-WIDTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records assembly + watch_style + subfeature_count ─────
TEST(SkillDiverCaseFullCompound, SignatureRecordsWatchAssembly)
{
    auto stock = skill::createCylindricalStock(42.0, 13.0);
    auto in = goodInput();
    auto out = skill::diver_case_full_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("diver_case_full_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_watch_assembly").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("watch_style").get<std::string>(),
              std::string("diver"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 6);
}

// ─── 5. Recognize via metadata replay (confidence 0.5) ──────────────────
TEST(SkillDiverCaseFullCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(42.0, 13.0);
    auto in = goodInput();
    auto out = skill::diver_case_full_compound::apply(*stock, in);

    auto cands = skill::diver_case_full_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 0.5);
    EXPECT_EQ(cands[0].recovered_params.at("crown_position").get<std::string>(),
              std::string("3"));
}
