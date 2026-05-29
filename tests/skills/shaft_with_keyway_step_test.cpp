// @lat: [[process/test-strategy#compound macro]]
//
// shaft_with_keyway_step — compound macro: step turn + sled keyway +
// retaining-ring groove.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/shaft_with_keyway_step.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::shaft_with_keyway_step::Input goodInput()
{
    skill::shaft_with_keyway_step::Input in;
    in.full_dia_mm        = 30.0;
    in.shaft_length_mm    = 100.0;
    in.step_z_mm          = 60.0;       // step starts 60 mm along Z
    in.step_dia_mm        = 20.0;
    in.key_w_mm           = 6.0;
    in.key_h_mm           = 3.0;
    in.key_len_mm         = 20.0;
    in.key_center_z_mm    = 30.0;       // mid of large-Ø section
    in.snap_ring_z_mm     = 80.0;       // on the small-Ø section
    in.snap_ring_w_mm     = 1.2;
    in.snap_ring_depth_mm = 0.5;
    return in;
}
}  // namespace

TEST(SkillShaftWithKeywayStep, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCylindricalStock(30.0, 100.0);
    const int faces0 = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::shaft_with_keyway_step::apply(*stock, in);

    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_LT(volumeOf(out.workpiece->shape()), volumeOf(stock->shape()));
    EXPECT_GT(out.workpiece->faceCount(), faces0);
}

TEST(SkillShaftWithKeywayStep, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(30.0, 100.0);

    // step_dia exceeds full_dia → no step.
    auto in = goodInput();
    in.step_dia_mm = 40.0;
    auto rep = skill::shaft_with_keyway_step::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
    bool found = false;
    for (const auto& f : rep.findings) if (f.code == "DFM-SHAFT-STEP") found = true;
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::shaft_with_keyway_step::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillShaftWithKeywayStep, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCylindricalStock(30.0, 100.0);
    auto in    = goodInput();
    auto out   = skill::shaft_with_keyway_step::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("shaft_with_keyway_step"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
}

TEST(SkillShaftWithKeywayStep, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(30.0, 100.0);
    auto in    = goodInput();
    auto out   = skill::shaft_with_keyway_step::apply(*stock, in);

    auto cands = skill::shaft_with_keyway_step::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params.at("step_dia_mm").get<double>(),
                20.0, 1e-6);
}

// Specific assertion — step_len + step_z = shaft_length identity.
TEST(SkillShaftWithKeywayStep, StepLenMatchesShaftIdentity)
{
    auto stock = skill::createCylindricalStock(30.0, 100.0);
    auto in    = goodInput();
    auto out   = skill::shaft_with_keyway_step::apply(*stock, in);

    const double recorded_step_len = out.signature.pattern.at("step_len_mm").get<double>();
    const double expected = in.shaft_length_mm - in.step_z_mm;
    EXPECT_NEAR(recorded_step_len, expected, 1e-6);

    // Snap ring must be on the small-Ø section: snap_ring_z > step_z.
    EXPECT_GT(in.snap_ring_z_mm, in.step_z_mm);
    // Step Ø < full Ø.
    EXPECT_LT(in.step_dia_mm, in.full_dia_mm);
}
