// @lat: [[process/test-strategy#medical dental emergence taper]]
//
// dental_implant_emergence_taper — conical emergence + axial screw bore.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/dental_implant_emergence_taper.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. apply produces compound (taper + bore) ──────────────────────────
TEST(SkillDentalEmergence, ApplyProducesCompoundShape)
{
    // 6 mm dia × 14 mm long abutment cylindrical stock
    auto stock = skill::createCylindricalStock(6.0, 14.0);
    const double stockVol = volumeOf(stock->shape());

    skill::dental_implant_emergence_taper::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.implant_axis_dir    = gp_Dir(0, 0, -1);
    in.implant_axis_origin = gp_Pnt(0.0, 0.0, 14.0);
    in.abutment_top_dia_mm    = 5.0;
    in.abutment_bottom_dia_mm = 3.5;
    in.transition_length_mm   = 3.0;
    in.screw_thread_size      = "M3";

    auto out = skill::dental_implant_emergence_taper::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Significant material removed (cone frustum + bore)
    const double removed = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 30.0);

    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("medical_feature_type").get<std::string>(),
              std::string("dental_emergence_profile"));
    EXPECT_EQ(out.signature.pattern.at("screw_thread_size").get<std::string>(),
              std::string("M3"));
    // M3 pilot per iso_thread_table = 2.5 mm
    EXPECT_NEAR(out.signature.pattern.at("screw_pilot_dia_mm").get<double>(),
                2.5, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("screw_thread_pitch_mm").get<double>(),
                0.5, 1e-6);
}

// ─── 2. validate rejects top ≤ bottom (no taper) ────────────────────────
TEST(SkillDentalEmergence, ValidateRejectsNoTaper)
{
    auto stock = skill::createCylindricalStock(6.0, 14.0);

    skill::dental_implant_emergence_taper::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.implant_axis_dir    = gp_Dir(0, 0, -1);
    in.implant_axis_origin = gp_Pnt(0.0, 0.0, 14.0);
    in.abutment_top_dia_mm    = 3.5;
    in.abutment_bottom_dia_mm = 3.5;   // equal → no taper → reject
    in.transition_length_mm   = 3.0;
    in.screw_thread_size      = "M3";

    auto r = skill::dental_implant_emergence_taper::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DENTAL-TAPER") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::dental_implant_emergence_taper::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. validate rejects unknown thread size ─────────────────────────────
TEST(SkillDentalEmergence, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCylindricalStock(6.0, 14.0);

    skill::dental_implant_emergence_taper::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.implant_axis_dir    = gp_Dir(0, 0, -1);
    in.implant_axis_origin = gp_Pnt(0.0, 0.0, 14.0);
    in.abutment_top_dia_mm    = 5.0;
    in.abutment_bottom_dia_mm = 3.5;
    in.transition_length_mm   = 3.0;
    in.screw_thread_size      = "M99";   // not in table

    auto r = skill::dental_implant_emergence_taper::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DENTAL-THREAD") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. validate rejects bottom_dia ≤ pilot ──────────────────────────────
TEST(SkillDentalEmergence, ValidateRejectsBoreLargerThanBottom)
{
    auto stock = skill::createCylindricalStock(6.0, 14.0);

    skill::dental_implant_emergence_taper::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.implant_axis_dir    = gp_Dir(0, 0, -1);
    in.implant_axis_origin = gp_Pnt(0.0, 0.0, 14.0);
    in.abutment_top_dia_mm    = 5.0;
    in.abutment_bottom_dia_mm = 2.0;        // ≤ M3 pilot 2.5 → reject
    in.transition_length_mm   = 3.0;
    in.screw_thread_size      = "M3";

    auto r = skill::dental_implant_emergence_taper::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DENTAL-THREAD") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 5. recognize replays metadata at confidence 1.0 ─────────────────────
TEST(SkillDentalEmergence, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(6.0, 14.0);

    skill::dental_implant_emergence_taper::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.implant_axis_dir    = gp_Dir(0, 0, -1);
    in.implant_axis_origin = gp_Pnt(0.0, 0.0, 14.0);
    in.abutment_top_dia_mm    = 5.0;
    in.abutment_bottom_dia_mm = 3.5;
    in.transition_length_mm   = 3.0;
    in.screw_thread_size      = "M3";

    auto out  = skill::dental_implant_emergence_taper::apply(*stock, in);
    auto cands = skill::dental_implant_emergence_taper::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("screw_thread_size").get<std::string>(),
              std::string("M3"));
}
