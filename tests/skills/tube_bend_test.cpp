// @lat: [[process/test-strategy#skill round-trip]]
//
// tube_bend — bend a straight tube; metadata-only.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tube_bend.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. ApplyHandlesInput — metadata-only, geometry unchanged ────────────
TEST(SkillTubeBend, ApplyHandlesInput)
{
    auto stock = skill::createCylindricalStock(20.0, 80.0);   // Ø20, length 80
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::tube_bend::Input in;
    in.bend_angle_deg     = 45.0;
    in.bend_radius_mm     = 30.0;    // CLR ≥ 1.5×R = 15
    in.axial_position_mm  = 40.0;
    auto out = skill::tube_bend::apply(*stock, in);

    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("tube_bend"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. ValidateRejectsBadInput — bend radius below CLR rule ─────────────
TEST(SkillTubeBend, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(20.0, 80.0);   // outerR = 10

    skill::tube_bend::Input in;
    in.bend_angle_deg     = 90.0;
    in.bend_radius_mm     = 10.0;     // < 1.5 × 10 = 15 → fail
    in.axial_position_mm  = 40.0;

    auto r = skill::tube_bend::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TUBE-BEND") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::tube_bend::apply(*stock, in), skill::SkillError);
}

// ─── 3. SignatureRecordsKind + key params ────────────────────────────────
TEST(SkillTubeBend, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(20.0, 80.0);

    skill::tube_bend::Input in;
    in.bend_angle_deg     = 60.0;
    in.bend_radius_mm     = 25.0;
    in.axial_position_mm  = 40.0;
    auto out = skill::tube_bend::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("tube_bend"));
    EXPECT_NEAR(out.signature.pattern["bend_angle_deg"].get<double>(),
                60.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["bend_radius_mm"].get<double>(),
                25.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["axial_position_mm"].get<double>(),
                40.0, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("draw_bender"));
}

// ─── 4. RecognizeMetadataReplay — from feature history; nothing geometric
TEST(SkillTubeBend, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(20.0, 80.0);

    skill::tube_bend::Input in;
    in.bend_angle_deg     = 90.0;
    in.bend_radius_mm     = 30.0;
    in.axial_position_mm  = 40.0;
    auto out = skill::tube_bend::apply(*stock, in);

    auto cands = skill::tube_bend::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["bend_angle_deg"].get<double>(),
                90.0, 1e-6);

    // Stripped of metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::tube_bend::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: angle out of (0,180] rejected with DFM-INPUT ───────────
TEST(SkillTubeBend, ValidateRejectsOutOfRangeAngle)
{
    auto stock = skill::createCylindricalStock(20.0, 80.0);

    skill::tube_bend::Input in;
    in.bend_angle_deg     = 200.0;   // > 180 → fail
    in.bend_radius_mm     = 30.0;
    in.axial_position_mm  = 40.0;

    auto r = skill::tube_bend::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);
}
