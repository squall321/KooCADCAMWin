// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/captive_screw_pocket_spec.hpp"

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

skill::captive_screw_pocket_spec::Input goodInput()
{
    skill::captive_screw_pocket_spec::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.axis_dir         = gp_Dir(0, 0, -1);
    in.thread_size      = "M6";
    in.chamfer_size_mm  = 0.5;
    in.groove_offset_mm = 2.0;
    return in;
}

}  // namespace

// ─── T1: REAL volume — clearance through + chamfer + groove annulus ─────
TEST(SkillCaptiveScrewPocketSpec, ApplyRemovesAllThreeCuts)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    auto out = skill::captive_screw_pocket_spec::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double removed = volBefore - volumeOf(out.workpiece->shape());

    // M6: clr = 6.6, groove_dia = 7.4, groove_width = 0.8.
    const double clrR    = 6.6 / 2.0;
    const double grooveR = 7.4 / 2.0;
    const double cylVol  = M_PI * clrR * clrR * 10.0;
    const double grooveVol = M_PI * (grooveR * grooveR - clrR * clrR) * 0.8;
    const double expected = cylVol + grooveVol;
    EXPECT_NEAR(removed, expected, expected * 0.20);
    EXPECT_GT(removed, cylVol);   // groove + chamfer present
}

// ─── T2: unknown spec rejected ──────────────────────────────────────────
TEST(SkillCaptiveScrewPocketSpec, ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in = goodInput();
    in.thread_size = "M0.5";
    auto r = skill::captive_screw_pocket_spec::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-METRIC-UNKNOWN"));
    EXPECT_THROW(skill::captive_screw_pocket_spec::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3: signature carries spec + is_compound=true ──────────────────────
TEST(SkillCaptiveScrewPocketSpec, SignatureCarriesSpec)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out   = skill::captive_screw_pocket_spec::apply(*stock, goodInput());
    EXPECT_EQ(out.signature.pattern["kind"],
              std::string("captive_screw_pocket_spec"));
    EXPECT_EQ(out.signature.pattern["is_compound"], true);
    EXPECT_EQ(out.signature.pattern["subfeature_count"], 3);
    EXPECT_EQ(out.signature.pattern["thread_size"], std::string("M6"));
    EXPECT_NEAR(out.signature.pattern["clearance_dia_mm"].get<double>(), 6.6, 1e-3);
    EXPECT_NEAR(out.signature.pattern["groove_dia_mm"].get<double>(),    7.4, 1e-3);
    EXPECT_NEAR(out.signature.pattern["groove_width_mm"].get<double>(),  0.8, 1e-3);
}

// ─── T4: recognize finds the compound (geometric coaxial-pair heuristic) ─
TEST(SkillCaptiveScrewPocketSpec, RecognizeFindsCaptivePocket)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out   = skill::captive_screw_pocket_spec::apply(*stock, goodInput());

    auto cands = skill::captive_screw_pocket_spec::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().recovered_params["thread_size"], std::string("M6"));
}

// ─── T5: DIN 6799 table values ──────────────────────────────────────────
TEST(SkillCaptiveScrewPocketSpec, Din6799TableValues)
{
    using namespace skill::captive_screw_pocket_spec;
    EXPECT_NEAR(clearanceDiameterFor("M3"),  3.4, 1e-9);
    EXPECT_NEAR(grooveDiameterFor("M3"),     4.0, 1e-9);
    EXPECT_NEAR(grooveWidthFor("M3"),        0.4, 1e-9);

    EXPECT_NEAR(clearanceDiameterFor("M6"),  6.6, 1e-9);
    EXPECT_NEAR(grooveDiameterFor("M6"),     7.4, 1e-9);
    EXPECT_NEAR(grooveWidthFor("M6"),        0.8, 1e-9);

    EXPECT_NEAR(clearanceDiameterFor("M16"), 18.0, 1e-9);
    EXPECT_NEAR(grooveDiameterFor("M16"),    19.0, 1e-9);
    EXPECT_NEAR(grooveWidthFor("M16"),        1.6, 1e-9);

    EXPECT_DOUBLE_EQ(clearanceDiameterFor("M99"), 0.0);
    EXPECT_DOUBLE_EQ(grooveDiameterFor("M99"),    0.0);
    EXPECT_DOUBLE_EQ(grooveWidthFor("M99"),       0.0);
}
