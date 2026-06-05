// @lat: [[process/test-strategy#compound pocket_watch_hunter_case_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pocket_watch_hunter_case_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::pocket_watch_hunter_case_compound::Input goodInput() {
    skill::pocket_watch_hunter_case_compound::Input in;
    in.case_dia_mm           = 50.0;
    in.case_thickness_mm     = 14.0;
    in.hinge_position        = "12_oclock";
    in.bow_attachment_dia_mm = 8.0;
    return in;
}
}  // namespace

// ─── 1. Apply produces valid shape (bow adds + hinge + glass + caseback cut) ─
TEST(SkillPocketWatchHunterCaseCompound, ApplyProducesValidShape)
{
    auto stock = skill::createCylindricalStock(50.0, 14.0);

    auto in = goodInput();
    auto out = skill::pocket_watch_hunter_case_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(stock->shape());
    const double v2 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v2, v1 * 0.4);
    EXPECT_LT(v2, v1 * 1.5);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects bad case_dia ───────────────────────────────────
TEST(SkillPocketWatchHunterCaseCompound, ValidateRejectsBadCaseDia)
{
    auto stock = skill::createCylindricalStock(50.0, 14.0);
    auto in = goodInput();
    in.case_dia_mm = 30.0;
    auto r = skill::pocket_watch_hunter_case_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CASE-DIA") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Validate rejects bad hinge_position ─────────────────────────────
TEST(SkillPocketWatchHunterCaseCompound, ValidateRejectsBadHinge)
{
    auto stock = skill::createCylindricalStock(50.0, 14.0);
    auto in = goodInput();
    in.hinge_position = "6_oclock";
    auto r = skill::pocket_watch_hunter_case_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-HINGE-POS") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records pocket_hunter style + 6 subfeatures ───────────
TEST(SkillPocketWatchHunterCaseCompound, SignatureRecordsWatchAssembly)
{
    auto stock = skill::createCylindricalStock(50.0, 14.0);
    auto in = goodInput();
    auto out = skill::pocket_watch_hunter_case_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("pocket_watch_hunter_case_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_watch_assembly").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("watch_style").get<std::string>(),
              std::string("pocket_hunter"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 6);
}

// ─── 5. Recognize via metadata replay ───────────────────────────────────
TEST(SkillPocketWatchHunterCaseCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(50.0, 14.0);
    auto in = goodInput();
    auto out = skill::pocket_watch_hunter_case_compound::apply(*stock, in);

    auto cands = skill::pocket_watch_hunter_case_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 0.5);
    EXPECT_EQ(cands[0].recovered_params.at("hinge_position").get<std::string>(),
              std::string("12_oclock"));
}
