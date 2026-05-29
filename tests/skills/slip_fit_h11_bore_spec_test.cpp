// @lat: [[process/test-strategy#skill round-trip]]
//
// slip_fit_h11_bore_spec — 5 tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/slip_fit_h11_bore_spec.hpp"

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

// ─── T1: apply removes volume ≈ oversize cylinder ────────────────────────
TEST(SkillSlipFitH11BoreSpec, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::slip_fit_h11_bore_spec::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.nominal_dia_mm = 18.0;     // H11 row 10–18 → +110 µm
    in.depth_mm       = 8.0;
    in.chamfer_mm     = 0.5;

    auto out = skill::slip_fit_h11_bore_spec::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double upperDev = 110.0;  // µm
    const double finalDia = in.nominal_dia_mm + (upperDev * 1e-3) / 2.0;
    const double rF = finalDia / 2.0;
    const double approx = M_PI * rF * rF * in.depth_mm;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.20);
    EXPECT_EQ(out.signature.skill_id, std::string("slip_fit_h11_bore_spec"));
}

// ─── T2: validate rejects unknown spec_key + apply throws ────────────────
TEST(SkillSlipFitH11BoreSpec, ValidateRejectsUnknownSpecKey)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::slip_fit_h11_bore_spec::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 10.0;
    in.depth_mm       = 5.0;
    in.spec_key       = "H7";   // not allowed

    auto r = skill::slip_fit_h11_bore_spec::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPEC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::slip_fit_h11_bore_spec::apply(*stock, in), skill::SkillError);
}

// ─── T3: signature carries H11 spec_key + is_compound=true ───────────────
TEST(SkillSlipFitH11BoreSpec, SignatureCarriesSpecAndCompoundFlag)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::slip_fit_h11_bore_spec::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 10.0;
    in.depth_mm       = 5.0;

    auto out = skill::slip_fit_h11_bore_spec::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("slip_fit_h11_bore_spec"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["spec_key"].get<std::string>(), std::string("H11"));
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 3);
}

// ─── T4: recognize returns ≥1 H11 candidate ──────────────────────────────
TEST(SkillSlipFitH11BoreSpec, RecognizeFindsH11Candidate)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::slip_fit_h11_bore_spec::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.nominal_dia_mm = 18.0;
    in.depth_mm       = 8.0;
    in.chamfer_mm     = 0.0;

    auto out = skill::slip_fit_h11_bore_spec::apply(*stock, in);
    auto cands = skill::slip_fit_h11_bore_spec::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    bool foundH11 = false;
    for (const auto& c : cands) {
        if (c.skill_id != "slip_fit_h11_bore_spec") continue;
        if (c.recovered_params.value("spec_key", std::string()) == "H11") {
            foundH11 = true;
            EXPECT_GT(c.confidence, 0.6);
        }
    }
    EXPECT_TRUE(foundH11);
}

// ─── T5: H11 deviation matches ISO 286-1 ─────────────────────────────────
TEST(SkillSlipFitH11BoreSpec, SpecTableMatchesIsoTable)
{
    // H11: Ø 6 → +75, Ø 18 → +110, Ø 50 → +160 µm.
    EXPECT_NEAR(skill::slip_fit_h11_bore_spec::upperDeviationMicronsH11(6.0),    75.0, 0.01);
    EXPECT_NEAR(skill::slip_fit_h11_bore_spec::upperDeviationMicronsH11(18.0), 110.0, 0.01);
    EXPECT_NEAR(skill::slip_fit_h11_bore_spec::upperDeviationMicronsH11(50.0), 160.0, 0.01);
    EXPECT_LT(skill::slip_fit_h11_bore_spec::upperDeviationMicronsH11(500.0),  0.0);
}
