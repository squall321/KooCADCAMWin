// @lat: [[process/test-strategy#skill round-trip]]
//
// penetrant_weld — PT NDT skill, metadata-only sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/penetrant_weld.hpp"

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

}  // namespace

// ─── 1. Apply: geometry unchanged ────────────────────────────────────────
TEST(SkillPenetrantWeld, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "aluminum_6061");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::penetrant_weld::Input in;
    in.weld_id           = "W1";
    in.sensitivity_class = "level_3";
    in.accept            = true;

    auto out = skill::penetrant_weld::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillPenetrantWeld, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "aluminum_6061");

    skill::penetrant_weld::Input in;
    in.weld_id           = "";        // empty
    in.sensitivity_class = "level_9"; // invalid

    auto r = skill::penetrant_weld::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::penetrant_weld::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillPenetrantWeld, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "aluminum_6061");

    skill::penetrant_weld::Input in;
    in.weld_id           = "WELD-P4";
    in.sensitivity_class = "level_4";
    in.accept            = false;

    auto out = skill::penetrant_weld::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("penetrant_weld"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("penetrant_weld"));
    EXPECT_EQ(out.signature.pattern["weld_id"].get<std::string>(),
              std::string("WELD-P4"));
    EXPECT_EQ(out.signature.pattern["sensitivity_class"].get<std::string>(),
              std::string("level_4"));
    EXPECT_FALSE(out.signature.pattern["accept"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillPenetrantWeld, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "aluminum_6061");

    skill::penetrant_weld::Input in;
    in.weld_id           = "WELD-P1";
    in.sensitivity_class = "level_2";
    in.accept            = true;
    auto out = skill::penetrant_weld::apply(*stock, in);

    auto cands = skill::penetrant_weld::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["sensitivity_class"].get<std::string>(),
              std::string("level_2"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::penetrant_weld::recognize(raw).empty());
}

// ─── 5. Skill-specific: all 5 sensitivity classes accepted ───────────────
TEST(SkillPenetrantWeld, AllFiveSensitivityClassesAccepted)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "aluminum_6061");

    const std::string levels[] = {
        "level_1", "level_2", "level_3", "level_4", "level_5"
    };
    for (const auto& lv : levels) {
        skill::penetrant_weld::Input in;
        in.weld_id           = "W";
        in.sensitivity_class = lv;
        in.accept            = true;

        auto r = skill::penetrant_weld::validate(*stock, in);
        EXPECT_TRUE(r.passed) << "level=" << lv;
    }
    EXPECT_EQ(std::size(levels), 5u);
}
