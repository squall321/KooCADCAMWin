// @lat: [[process/test-strategy#skill round-trip]]
//
// form_tap skill tests (slice — threading variant).
//   1. Apply runs a real helical sweep, removes a small amount of stock.
//   2. Form-tap pilot chart is DIFFERENT from cutting-tap chart.
//   3. DFM emits info-level DFM-FORM-TAP-MAT on non-ductile material.
//   4. DFM rejects insufficient engagement.
//   5. Recognize replays signature from feature history.
//   6. Ductile-material classification helper.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
#include "skills/form_tap.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply on ductile material: thread cuts a small volume ─────────────
TEST(SkillFormTap, ApplyOnDuctileMaterial)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "aluminum_6061");

    // Form-tap pilot for M3 = 2.75 mm (larger than the 2.5 mm cutting pilot).
    skill::drill_hole::Input drill;
    drill.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    drill.position_x_mm = 25.0;
    drill.position_y_mm = 25.0;
    drill.diameter_mm   = 2.75;
    drill.depth_mm      = 6.0;
    auto pilot = skill::drill_hole::apply(*stock, drill);

    const double volBefore = volumeOf(pilot.workpiece->shape());

    skill::form_tap::Input in;
    in.existing_hole_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25.0, 25.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size     = "M3";
    in.pitch_mm        = 0.5;
    in.thread_depth_mm = 4.0;

    auto out = skill::form_tap::apply(*pilot.workpiece, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_LE(volumeOf(out.workpiece->shape()), volBefore + 1e-6);
    EXPECT_EQ(out.signature.skill_id, std::string("form_tap"));
    EXPECT_EQ(out.signature.pattern.at("process").get<std::string>(),
              std::string("cold_forming_chipless"));
    EXPECT_TRUE(out.signature.pattern.at("material_ductile").get<bool>());
    // Form taps are flute-less.
    EXPECT_EQ(out.signature.tooling.flute_count, 0);
}

// ── 2. Form-tap pilot chart values DIFFER from cutting-tap chart ─────────
TEST(SkillFormTap, FormTapPilotChartDiffers)
{
    // M3 cutting-tap pilot is 2.5 mm; form-tap is ≈ 3.0 - 0.5×0.5 = 2.75.
    EXPECT_NEAR(skill::form_tap::formTapPilotDiameter("M3"), 2.75, 1e-3);
    // M4: cutting = 3.3, form ≈ 4 - 0.5×0.7 = 3.65.
    EXPECT_NEAR(skill::form_tap::formTapPilotDiameter("M4"), 3.65, 1e-3);
    // M6: cutting = 5.0, form ≈ 6 - 0.5×1.0 = 5.5.
    EXPECT_NEAR(skill::form_tap::formTapPilotDiameter("M6"), 5.50, 1e-3);
    // Unknown size.
    EXPECT_NEAR(skill::form_tap::formTapPilotDiameter("M99"), 0.0, 1e-6);
}

// ── 3. DFM emits info on non-ductile material ────────────────────────────
TEST(SkillFormTap, NonDuctileMaterialEmitsInfo)
{
    // "cast_iron" is NOT in the ductile list → info finding.
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "cast_iron");

    skill::drill_hole::Input drill;
    drill.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    drill.position_x_mm = 25.0;
    drill.position_y_mm = 25.0;
    drill.diameter_mm   = 2.75;
    drill.depth_mm      = 6.0;
    auto pilot = skill::drill_hole::apply(*stock, drill);

    skill::form_tap::Input in;
    in.existing_hole_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25.0, 25.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size     = "M3";
    in.pitch_mm        = 0.5;
    in.thread_depth_mm = 4.0;

    auto r = skill::form_tap::validate(*pilot.workpiece, in);
    // Info-level only; report still passes.
    EXPECT_TRUE(r.passed);

    bool found = false;
    std::string severity;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FORM-TAP-MAT") {
            found = true;
            severity = f.severity;
        }
    EXPECT_TRUE(found);
    EXPECT_EQ(severity, std::string("info"));
}

// ── 4. DFM rejects shallow engagement ────────────────────────────────────
TEST(SkillFormTap, ValidateRejectsShallowEngagement)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "aluminum_6061");

    skill::form_tap::Input in;
    in.existing_hole_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.thread_size     = "M3";
    in.pitch_mm        = 0.5;
    in.thread_depth_mm = 0.5;    // < 1.5 × 0.5

    auto r = skill::form_tap::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TAP-ENGAGE") found = true;
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::form_tap::apply(*stock, in), skill::SkillError);
}

// ── 5. Recognize replays signature ───────────────────────────────────────
TEST(SkillFormTap, RecognizeReplaysSignature)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "aluminum_6061");

    skill::drill_hole::Input drill;
    drill.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    drill.position_x_mm = 25.0;
    drill.position_y_mm = 25.0;
    drill.diameter_mm   = 3.65;
    drill.depth_mm      = 6.0;
    auto pilot = skill::drill_hole::apply(*stock, drill);

    skill::form_tap::Input in;
    in.existing_hole_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25.0, 25.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size     = "M4";
    in.pitch_mm        = 0.7;
    in.thread_depth_mm = 5.0;

    auto out   = skill::form_tap::apply(*pilot.workpiece, in);
    auto cands = skill::form_tap::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("form_tap"));
    EXPECT_EQ(cands[0].recovered_params["thread_size"], "M4");
    EXPECT_NEAR(cands[0].confidence, 0.5, 1e-6);
}

// ── 6. Ductile-material classifier ───────────────────────────────────────
TEST(SkillFormTap, DuctileMaterialClassifier)
{
    EXPECT_TRUE (skill::form_tap::isDuctileMaterial("aluminum_6061"));
    EXPECT_TRUE (skill::form_tap::isDuctileMaterial("copper"));
    EXPECT_TRUE (skill::form_tap::isDuctileMaterial("brass"));
    EXPECT_TRUE (skill::form_tap::isDuctileMaterial("mild_steel"));
    EXPECT_FALSE(skill::form_tap::isDuctileMaterial("cast_iron"));
    EXPECT_FALSE(skill::form_tap::isDuctileMaterial("titanium"));
    EXPECT_FALSE(skill::form_tap::isDuctileMaterial("hardened_4140"));
    EXPECT_FALSE(skill::form_tap::isDuctileMaterial(""));
}
