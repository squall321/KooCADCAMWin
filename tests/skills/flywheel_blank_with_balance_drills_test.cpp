// @lat: [[process/test-strategy#compound macro]]
//
// flywheel_blank_with_balance_drills — compound macro: central bore +
// N radially equispaced blind balance-correction holes.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/flywheel_blank_with_balance_drills.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::flywheel_blank_with_balance_drills::Input goodInput()
{
    skill::flywheel_blank_with_balance_drills::Input in;
    in.outer_dia_mm           = 200.0;
    in.thickness_mm           = 25.0;
    in.bore_dia_mm            = 30.0;
    in.balance_drill_count    = 6;
    in.balance_drill_dia_mm   = 6.0;
    in.balance_drill_depth_mm = 5.0;
    in.hole_radius_mm         = 80.0;     // 0.85 · R_out ≈ 85; pick 80
    in.angle_offset_deg       = 0.0;
    return in;
}
}  // namespace

TEST(SkillFlywheelBlankWithBalanceDrills, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCylindricalStock(200.0, 25.0);
    const int faces0 = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::flywheel_blank_with_balance_drills::apply(*stock, in);

    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume delta should be ≈ bore_vol + N·drill_vol  (within 10% slack
    // since we don't model cone tip etc.).
    const double bRad   = in.bore_dia_mm / 2.0;
    const double drillR = in.balance_drill_dia_mm / 2.0;
    const double expected = M_PI * bRad * bRad * in.thickness_mm +
                            in.balance_drill_count *
                            M_PI * drillR * drillR * in.balance_drill_depth_mm;
    const double diff = volumeOf(stock->shape()) -
                        volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, expected, expected * 0.10);

    EXPECT_GT(out.workpiece->faceCount(), faces0);
}

TEST(SkillFlywheelBlankWithBalanceDrills, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(200.0, 25.0);

    // drill depth >= thickness → would exit far face.
    auto in = goodInput();
    in.balance_drill_depth_mm = 30.0;
    auto rep = skill::flywheel_blank_with_balance_drills::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
    bool found = false;
    for (const auto& f : rep.findings) if (f.code == "DFM-FLYWHEEL-DEPTH") found = true;
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::flywheel_blank_with_balance_drills::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillFlywheelBlankWithBalanceDrills, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCylindricalStock(200.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::flywheel_blank_with_balance_drills::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("flywheel_blank_with_balance_drills"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    // bore + 6 drills = 7
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 7);
    EXPECT_EQ(out.signature.pattern.at("balance_drill_count").get<int>(), 6);
}

TEST(SkillFlywheelBlankWithBalanceDrills, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(200.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::flywheel_blank_with_balance_drills::apply(*stock, in);

    auto cands = skill::flywheel_blank_with_balance_drills::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params.at("balance_drill_count").get<int>(), 6);
}

// Specific assertion — N equispaced holes leave a pattern that exactly
// satisfies (subfeature_count - 1) == balance_drill_count (the bore is the
// "+1"), and the recorded hole positions sit on the pitch circle.
TEST(SkillFlywheelBlankWithBalanceDrills, HolePositionsOnPitchCircle)
{
    auto stock = skill::createCylindricalStock(200.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::flywheel_blank_with_balance_drills::apply(*stock, in);

    const int subCount = out.signature.pattern.at("subfeature_count").get<int>();
    EXPECT_EQ(subCount - 1, in.balance_drill_count);

    const auto& positions = out.signature.pattern.at("hole_positions");
    ASSERT_EQ(positions.size(), static_cast<size_t>(in.balance_drill_count));
    for (const auto& p : positions) {
        const double x = p.at("x").get<double>();
        const double y = p.at("y").get<double>();
        EXPECT_NEAR(std::hypot(x, y), in.hole_radius_mm, 1e-6);
    }
}
