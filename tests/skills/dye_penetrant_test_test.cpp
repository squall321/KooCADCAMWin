// @lat: [[process/test-strategy#skill round-trip]]
//
// dye_penetrant_test — PT inspection skill (metadata-only).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/dye_penetrant_test.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <gp_Dir.hxx>

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

// ── 1. Apply on any non-porous material clones geometry unchanged ────────
TEST(SkillDyePenetrantTest, ApplyClonesGeometryUnchanged)
{
    // PT works on any non-porous material — exercise on aluminum to show it.
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "aluminum_6061");
    const double volBefore = volumeOf(stock->shape());

    skill::dye_penetrant_test::Input in;
    in.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.dwell_time_min     = 10.0;
    in.sensitivity_class  = "level_2_med";

    auto out = skill::dye_penetrant_test::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM-INPUT: zero dwell_time and bad sensitivity_class rejected ─────
TEST(SkillDyePenetrantTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::dye_penetrant_test::Input badDwell;
    badDwell.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    badDwell.dwell_time_min     = 0.0;
    badDwell.sensitivity_class  = "level_2_med";
    auto r1 = skill::dye_penetrant_test::validate(*stock, badDwell);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::dye_penetrant_test::apply(*stock, badDwell),
                 skill::SkillError);

    skill::dye_penetrant_test::Input badSens;
    badSens.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    badSens.dwell_time_min     = 10.0;
    badSens.sensitivity_class  = "level_5_extreme";   // not allowed
    auto r2 = skill::dye_penetrant_test::validate(*stock, badSens);
    EXPECT_FALSE(r2.passed);
}

// ── 3. Under-soak / over-soak dwell emits INFO but does not block ────────
TEST(SkillDyePenetrantTest, DwellRangeInfoOnly)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::dye_penetrant_test::Input shortDwell;
    shortDwell.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    shortDwell.dwell_time_min     = 2.0;       // < 5 min → info
    shortDwell.sensitivity_class  = "level_2_med";
    auto r1 = skill::dye_penetrant_test::validate(*stock, shortDwell);
    EXPECT_TRUE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-PT-DWELL-SHORT"));
    EXPECT_NO_THROW(skill::dye_penetrant_test::apply(*stock, shortDwell));

    skill::dye_penetrant_test::Input longDwell = shortDwell;
    longDwell.dwell_time_min = 90.0;           // > 60 min → info
    auto r2 = skill::dye_penetrant_test::validate(*stock, longDwell);
    EXPECT_TRUE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-PT-DWELL-LONG"));
}

// ── 4. Signature records PT parameters + tooling ─────────────────────────
TEST(SkillDyePenetrantTest, SignatureRecordsParametersAndTooling)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "titanium_grade5");

    skill::dye_penetrant_test::Input in;
    in.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.dwell_time_min     = 15.0;
    in.sensitivity_class  = "level_3_high";

    auto out = skill::dye_penetrant_test::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("dye_penetrant_test"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("dye_penetrant_test"));
    EXPECT_NEAR(out.signature.pattern["dwell_time_min"].get<double>(),
                15.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["sensitivity_class"].get<std::string>(),
              std::string("level_3_high"));
    EXPECT_EQ(out.signature.pattern["ndt_category"].get<std::string>(),
              std::string("PT"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("penetrant_kit"));
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
}

// ── 5. Recognize metadata replay round-trip ──────────────────────────────
TEST(SkillDyePenetrantTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::dye_penetrant_test::Input in;
    in.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.dwell_time_min     = 10.0;
    in.sensitivity_class  = "level_2_med";

    auto out = skill::dye_penetrant_test::apply(*stock, in);

    auto cands = skill::dye_penetrant_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].skill_id, std::string("dye_penetrant_test"));
    EXPECT_NEAR(cands[0].recovered_params["dwell_time_min"].get<double>(),
                10.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape(), out.workpiece->material());
    EXPECT_TRUE(skill::dye_penetrant_test::recognize(raw).empty());
}
