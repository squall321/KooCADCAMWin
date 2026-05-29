// @lat: [[process/test-strategy#skill round-trip]]
//
// iso_h7_bore_spec skill — 5 tests covering apply / validate / signature /
// recognize / spec-table behaviour.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/iso_h7_bore_spec.hpp"

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

// ─── T1: apply() produces non-null shape and removes expected volume ─────
TEST(SkillIsoH7BoreSpec, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::iso_h7_bore_spec::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.nominal_dia_mm = 18.0;     // H7 row 10–18 → 18 µm dev
    in.depth_mm       = 10.0;
    in.chamfer_mm     = 0.3;

    auto out = skill::iso_h7_bore_spec::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected: ream cylinder volume (the larger of drill vs ream wins after
    // fuse-then-cut).  finalDia = 18.0 + 0.018/2 = 18.009 mm.
    const double upperDev = 18.0;  // µm
    const double finalDia = in.nominal_dia_mm + (upperDev * 1e-3) / 2.0;
    const double rf = finalDia / 2.0;
    const double cylVol = M_PI * rf * rf * in.depth_mm;
    // Chamfer adds a small annular frustum (negligible vs cylinder).
    const double approx = cylVol;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.20)
        << "iso_h7_bore_spec should remove ~ cylinder of final dia";
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("iso_h7_bore_spec"));
}

// ─── T2: validate rejects unknown spec_key + apply throws ────────────────
TEST(SkillIsoH7BoreSpec, ValidateRejectsUnknownSpecKey)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::iso_h7_bore_spec::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 10.0;
    in.depth_mm       = 5.0;
    in.spec_key       = "P7";   // wrong — this skill is H7-only

    auto r = skill::iso_h7_bore_spec::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPEC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::iso_h7_bore_spec::apply(*stock, in), skill::SkillError);
}

// ─── T3: signature carries spec_key + is_compound=true ───────────────────
TEST(SkillIsoH7BoreSpec, SignatureCarriesSpecAndCompoundFlag)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::iso_h7_bore_spec::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 10.0;
    in.depth_mm       = 5.0;

    auto out = skill::iso_h7_bore_spec::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("iso_h7_bore_spec"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["spec_key"].get<std::string>(),
              std::string("H7"));
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 3);
}

// ─── T4: recognize returns ≥1 candidate with H7 spec_key ─────────────────
TEST(SkillIsoH7BoreSpec, RecognizeFindsCandidate)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::iso_h7_bore_spec::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 18.0;
    in.depth_mm       = 10.0;
    in.chamfer_mm     = 0.0;     // skip chamfer to keep cyl pristine for recog

    auto out = skill::iso_h7_bore_spec::apply(*stock, in);
    auto cands = skill::iso_h7_bore_spec::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    bool foundH7 = false;
    for (const auto& c : cands) {
        if (c.skill_id != "iso_h7_bore_spec") continue;
        if (c.recovered_params.value("spec_key", std::string()) == "H7") {
            foundH7 = true;
            // Metadata-replay fallback should boost confidence near 0.95.
            EXPECT_GT(c.confidence, 0.7);
        }
    }
    EXPECT_TRUE(foundH7);
}

// ─── T5: spec-table lookup matches ISO 286-1 Table 2 values ──────────────
TEST(SkillIsoH7BoreSpec, SpecTableMatchesIsoTable)
{
    // ISO 286-1 H7: Ø 6 → 15 µm, Ø 18 → 18 µm, Ø 50 → 25 µm.
    EXPECT_NEAR(skill::iso_h7_bore_spec::upperDeviationMicrons(6.0),  15.0, 0.01);
    EXPECT_NEAR(skill::iso_h7_bore_spec::upperDeviationMicrons(18.0), 18.0, 0.01);
    EXPECT_NEAR(skill::iso_h7_bore_spec::upperDeviationMicrons(50.0), 25.0, 0.01);
    // Out of range → -1.
    EXPECT_LT(skill::iso_h7_bore_spec::upperDeviationMicrons(500.0), 0.0);
}
