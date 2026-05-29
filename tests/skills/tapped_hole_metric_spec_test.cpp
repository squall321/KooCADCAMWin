// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tapped_hole_metric_spec.hpp"

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

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

skill::tapped_hole_metric_spec::Input goodInput()
{
    skill::tapped_hole_metric_spec::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 25.0;
    in.position_y_mm     = 25.0;
    in.axis_dir          = gp_Dir(0, 0, -1);
    in.thread_size       = "M6";
    in.tap_depth_mm      = 8.0;
    in.through           = false;
    in.chamfer_size_mm   = 0.4;
    in.pilot_extra_depth_mm = 1.0;
    return in;
}

}  // namespace

// ─── T1: REAL volume — pilot column ±20% ────────────────────────────────
TEST(SkillTappedHoleMetricSpec, ApplyRemovesPilotPlusChamfer)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    const double volBefore = volumeOf(stock->shape());

    auto out = skill::tapped_hole_metric_spec::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = volBefore - volumeOf(out.workpiece->shape());

    // M6 pilot = 5.0 mm → R=2.5.  Pilot depth = 8 + 1 = 9 mm.
    const double pilotR = 5.0 / 2.0;
    const double pilotLen = 9.0;
    const double expected = M_PI * pilotR * pilotR * pilotLen;
    EXPECT_NEAR(removed, expected, expected * 0.20)
        << "pilot column volume should match within ±20%";
    EXPECT_GT(removed, expected);   // chamfer adds extra
}

// ─── T2: unknown spec rejected; apply throws ────────────────────────────
TEST(SkillTappedHoleMetricSpec, ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    auto in = goodInput();
    in.thread_size = "M77";
    auto r = skill::tapped_hole_metric_spec::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-METRIC-UNKNOWN"));
    EXPECT_THROW(skill::tapped_hole_metric_spec::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3: signature carries spec key + is_compound ───────────────────────
TEST(SkillTappedHoleMetricSpec, SignatureCarriesSpecKey)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    auto out   = skill::tapped_hole_metric_spec::apply(*stock, goodInput());
    EXPECT_EQ(out.signature.pattern["kind"], std::string("tapped_hole_metric_spec"));
    EXPECT_EQ(out.signature.pattern["is_compound"], true);
    EXPECT_EQ(out.signature.pattern["subfeature_count"], 2);
    EXPECT_EQ(out.signature.pattern["thread_size"], std::string("M6"));
    EXPECT_NEAR(out.signature.pattern["pilot_dia_mm"].get<double>(), 5.0, 1e-3);
}

// ─── T4: recognize returns ≥1 candidate ─────────────────────────────────
TEST(SkillTappedHoleMetricSpec, RecognizeFindsPilotMatchingIso965)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    auto out   = skill::tapped_hole_metric_spec::apply(*stock, goodInput());

    auto cands = skill::tapped_hole_metric_spec::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("tapped_hole_metric_spec"));
    EXPECT_EQ(cands.front().recovered_params["thread_size"], std::string("M6"));
}

// ─── T5: ISO 965 pilot table values are correct ─────────────────────────
TEST(SkillTappedHoleMetricSpec, Iso965TableValues)
{
    using skill::tapped_hole_metric_spec::pilotDiameterFor;
    EXPECT_NEAR(pilotDiameterFor("M3"),   2.50, 1e-9);
    EXPECT_NEAR(pilotDiameterFor("M4"),   3.30, 1e-9);
    EXPECT_NEAR(pilotDiameterFor("M6"),   5.00, 1e-9);
    EXPECT_NEAR(pilotDiameterFor("M10"),  8.50, 1e-9);
    EXPECT_NEAR(pilotDiameterFor("M16"), 14.00, 1e-9);
    EXPECT_NEAR(pilotDiameterFor("M30"), 26.50, 1e-9);
    EXPECT_DOUBLE_EQ(pilotDiameterFor("M99"), 0.0);
}
