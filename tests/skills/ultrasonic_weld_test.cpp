// @lat: [[process/test-strategy#skill round-trip]]
//
// ultrasonic_weld skill — 6-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ultrasonic_weld.hpp"

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

// ─── 1. Apply: shallow disc fuses, volume increases slightly ──────────────
TEST(SkillUltrasonicWeld, ApplyAddsShallowDisc)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 2.0);  // thin sheet
    const double volBefore = volumeOf(stock->shape());

    skill::ultrasonic_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm  = 20.0;
    in.position_y_mm  = 20.0;
    in.spot_dia_mm    = 4.0;
    in.bead_height_mm = 0.2;
    in.material       = "aluminum";
    in.sheet_thickness_mm = 2.0;

    auto out = skill::ultrasonic_weld::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approxAdded = M_PI * 2.0 * 2.0 * 0.2;
    const double added = volumeOf(out.workpiece->shape()) - volBefore;
    EXPECT_GT(added, approxAdded * 0.5);
    EXPECT_LT(added, approxAdded * 1.5);

    EXPECT_EQ(out.signature.skill_id, std::string("ultrasonic_weld"));
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("ultrasonic_horn"));
    EXPECT_EQ(out.signature.pattern["kind"], "ultrasonic_weld");
    EXPECT_TRUE(out.signature.pattern["solid_state"].get<bool>());
}

// ─── 2. DFM: spot_dia outside [1, 12] rejected ────────────────────────────
TEST(SkillUltrasonicWeld, ValidateRejectsBadDia)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 2.0);

    skill::ultrasonic_weld::Input tiny;
    tiny.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tiny.position_x_mm  = 20.0;
    tiny.position_y_mm  = 20.0;
    tiny.spot_dia_mm    = 0.5;       // < 1
    tiny.bead_height_mm = 0.1;
    auto r1 = skill::ultrasonic_weld::validate(*stock, tiny);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::ultrasonic_weld::apply(*stock, tiny),
                 skill::SkillError);

    skill::ultrasonic_weld::Input big = tiny;
    big.spot_dia_mm    = 15.0;       // > 12
    big.bead_height_mm = 0.2;
    auto r2 = skill::ultrasonic_weld::validate(*stock, big);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. DFM: bead_height > 0.5 mm rejected ────────────────────────────────
TEST(SkillUltrasonicWeld, ValidateRejectsDeepBead)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 2.0);

    skill::ultrasonic_weld::Input deep;
    deep.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    deep.position_x_mm  = 20.0;
    deep.position_y_mm  = 20.0;
    deep.spot_dia_mm    = 5.0;
    deep.bead_height_mm = 0.8;       // > 0.5

    auto r = skill::ultrasonic_weld::validate(*stock, deep);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-USW-HEIGHT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. DFM: sheet thicker than 3 mm → warning (not error) ────────────────
TEST(SkillUltrasonicWeld, ValidateThickSheetWarning)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::ultrasonic_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 20.0;
    in.position_y_mm  = 20.0;
    in.spot_dia_mm    = 4.0;
    in.bead_height_mm = 0.2;
    in.sheet_thickness_mm = 5.0;     // > 3 → warn, still passes

    auto r = skill::ultrasonic_weld::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool foundWarn = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-USW-THK" && f.severity == "warning") {
            foundWarn = true;
            break;
        }
    EXPECT_TRUE(foundWarn);
}

// ─── 5. Recognize via metadata ────────────────────────────────────────────
TEST(SkillUltrasonicWeld, RecognizeViaMetadata)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 2.0);

    skill::ultrasonic_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 20.0;
    in.position_y_mm  = 20.0;
    in.spot_dia_mm    = 6.0;
    in.bead_height_mm = 0.2;
    in.vibration_amplitude_um = 30.0;
    in.frequency_khz          = 20.0;

    auto out = skill::ultrasonic_weld::apply(*stock, in);
    auto cands = skill::ultrasonic_weld::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands[0].confidence, 0.9);
    EXPECT_NEAR(cands[0].recovered_params["spot_dia_mm"].get<double>(),
                6.0, 1e-3);
    EXPECT_NEAR(cands[0].recovered_params["bead_height_mm"].get<double>(),
                0.2, 1e-3);
    EXPECT_NEAR(cands[0].recovered_params["vibration_amplitude_um"].get<double>(),
                30.0, 1e-3);
}

// ─── 6. Recognize geometric — shallow disc detected on anonymous workpiece ─
TEST(SkillUltrasonicWeld, RecognizeGeometricShallow)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 2.0);

    skill::ultrasonic_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 20.0;
    in.position_y_mm  = 20.0;
    in.spot_dia_mm    = 5.0;
    in.bead_height_mm = 0.2;

    auto out = skill::ultrasonic_weld::apply(*stock, in);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands = skill::ultrasonic_weld::recognize(raw);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["spot_dia_mm"].get<double>(),
                5.0, 0.2);
    EXPECT_LT(cands[0].recovered_params["bead_height_mm"].get<double>(), 0.5);
}
