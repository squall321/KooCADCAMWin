// @lat: [[process/test-strategy#skill round-trip]]
//
// press_fit_p7_bore_spec — 5 tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/press_fit_p7_bore_spec.hpp"

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

// ─── T1: apply removes volume close to nominal drill cylinder ────────────
TEST(SkillPressFitP7BoreSpec, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::press_fit_p7_bore_spec::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.nominal_dia_mm = 20.0;       // P7 row 18–30: upper -14, lower -35 µm
    in.depth_mm       = 10.0;
    in.chamfer_mm     = 0.5;

    auto out = skill::press_fit_p7_bore_spec::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Drill at nominal dominates after fuse-then-cut (since drillDia >
    // finalDia for P7 — undersize bore).
    const double rD = in.nominal_dia_mm / 2.0;
    const double approx = M_PI * rD * rD * in.depth_mm;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.20)
        << "press_fit_p7 should remove ~ nominal-dia drill cylinder";
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("press_fit_p7_bore_spec"));
}

// ─── T2: validate rejects unknown spec_key + apply throws ────────────────
TEST(SkillPressFitP7BoreSpec, ValidateRejectsUnknownSpecKey)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::press_fit_p7_bore_spec::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 10.0;
    in.depth_mm       = 5.0;
    in.spec_key       = "H7";   // wrong — this skill is P7-only

    auto r = skill::press_fit_p7_bore_spec::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPEC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::press_fit_p7_bore_spec::apply(*stock, in), skill::SkillError);
}

// ─── T3: signature carries P7 spec_key + is_compound=true ────────────────
TEST(SkillPressFitP7BoreSpec, SignatureCarriesSpecAndCompoundFlag)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::press_fit_p7_bore_spec::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 10.0;
    in.depth_mm       = 5.0;

    auto out = skill::press_fit_p7_bore_spec::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("press_fit_p7_bore_spec"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["spec_key"].get<std::string>(), std::string("P7"));
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 3);
}

// ─── T4: recognize returns ≥1 P7 candidate ───────────────────────────────
TEST(SkillPressFitP7BoreSpec, RecognizeFindsP7Candidate)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::press_fit_p7_bore_spec::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.nominal_dia_mm = 20.0;
    in.depth_mm       = 10.0;
    in.chamfer_mm     = 0.0;

    auto out = skill::press_fit_p7_bore_spec::apply(*stock, in);
    auto cands = skill::press_fit_p7_bore_spec::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    bool foundP7 = false;
    for (const auto& c : cands) {
        if (c.skill_id != "press_fit_p7_bore_spec") continue;
        if (c.recovered_params.value("spec_key", std::string()) == "P7") {
            foundP7 = true;
            EXPECT_GT(c.confidence, 0.6);
        }
    }
    EXPECT_TRUE(foundP7);
}

// ─── T5: P7 table values match ISO 286-1 ─────────────────────────────────
TEST(SkillPressFitP7BoreSpec, SpecTableMatchesIsoTable)
{
    // Ø 10 → upper -9, lower -24 µm.
    auto d10 = skill::press_fit_p7_bore_spec::deviationFor(10.0);
    EXPECT_TRUE(d10.valid);
    EXPECT_NEAR(d10.upper_um,  -9.0, 0.01);
    EXPECT_NEAR(d10.lower_um, -24.0, 0.01);

    // Ø 18 → upper -11, lower -29 µm (row 10–18).
    auto d18 = skill::press_fit_p7_bore_spec::deviationFor(18.0);
    EXPECT_TRUE(d18.valid);
    EXPECT_NEAR(d18.upper_um, -11.0, 0.01);
    EXPECT_NEAR(d18.lower_um, -29.0, 0.01);

    // Ø 50 → upper -17, lower -42 µm (row 30–50).
    auto d50 = skill::press_fit_p7_bore_spec::deviationFor(50.0);
    EXPECT_TRUE(d50.valid);
    EXPECT_NEAR(d50.upper_um, -17.0, 0.01);
    EXPECT_NEAR(d50.lower_um, -42.0, 0.01);

    // Out of range.
    auto d999 = skill::press_fit_p7_bore_spec::deviationFor(999.0);
    EXPECT_FALSE(d999.valid);
}
