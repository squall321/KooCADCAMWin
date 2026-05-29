// @lat: [[process/test-strategy#skill round-trip]]
//
// bolt_hole_metric_spec — verifies the ISO 273 spec table lookup, the
// 2-cut compound geometry (clearance through-hole + entry chamfer), and
// the recognize() heuristic that reverse-maps measured diameter → size.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bolt_hole_metric_spec.hpp"

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

skill::bolt_hole_metric_spec::Input goodInput()
{
    skill::bolt_hole_metric_spec::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.axis_dir         = gp_Dir(0, 0, -1);
    in.thread_size      = "M6";
    in.fit_class        = "medium";
    in.chamfer_size_mm  = 0.5;
    in.chamfer_angle_deg = 30.0;
    return in;
}

}  // namespace

// ─── T1: REAL geometry — volume removed matches clearance cyl + chamfer cone
TEST(SkillBoltHoleMetricSpec, ApplyProducesValidGeometryAndExpectedVolume)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    auto out = skill::bolt_hole_metric_spec::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    const double removed  = volBefore - volAfter;

    // Expected: M6 medium = 6.6 mm clearance → R=3.3.  Through-hole 10 mm.
    // Plus chamfer: leg=0.5, depth=0.5/tan(30°)≈0.866, cone volume minus
    // overlapping cylinder portion ≈ small correction.
    const double clrR     = 6.6 / 2.0;
    const double through  = 10.0;
    const double cylVol   = M_PI * clrR * clrR * through;
    EXPECT_NEAR(removed, cylVol, cylVol * 0.20)
        << "should remove clearance through-hole + chamfer (±20%)";
    EXPECT_GT(removed, cylVol)  // chamfer adds a bit
        << "compound is cyl + chamfer cone, must remove MORE than cyl alone";

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("bolt_hole_metric_spec"));
}

// ─── T2: validate rejects unknown thread_size; apply throws ─────────────
TEST(SkillBoltHoleMetricSpec, ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    auto in = goodInput();
    in.thread_size = "M99";   // not in ISO 273 table

    auto r = skill::bolt_hole_metric_spec::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-METRIC-UNKNOWN"));
    EXPECT_THROW(skill::bolt_hole_metric_spec::apply(*stock, in),
                 skill::SkillError);

    // Also reject unknown fit_class
    auto in2 = goodInput();
    in2.fit_class = "snug";
    auto r2 = skill::bolt_hole_metric_spec::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-METRIC-FIT"));
}

// ─── T3: signature carries spec key + is_compound ──────────────────────
TEST(SkillBoltHoleMetricSpec, SignatureCarriesSpecKey)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out   = skill::bolt_hole_metric_spec::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.pattern["kind"], std::string("bolt_hole_metric_spec"));
    EXPECT_EQ(out.signature.pattern["is_compound"], true);
    EXPECT_EQ(out.signature.pattern["subfeature_count"], 2);
    EXPECT_EQ(out.signature.pattern["thread_size"], std::string("M6"));
    EXPECT_EQ(out.signature.pattern["fit_class"], std::string("medium"));
    EXPECT_NEAR(out.signature.pattern["clearance_dia_mm"].get<double>(), 6.6, 1e-3);
    EXPECT_NEAR(out.signature.params["clearance_dia_mm"].get<double>(), 6.6, 1e-3);
}

// ─── T4: recognize returns ≥1 candidate with the right spec key ─────────
TEST(SkillBoltHoleMetricSpec, RecognizeMatchesIso273Diameter)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out   = skill::bolt_hole_metric_spec::apply(*stock, goodInput());

    auto cands = skill::bolt_hole_metric_spec::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("bolt_hole_metric_spec"));
    EXPECT_EQ(c.recovered_params["thread_size"], std::string("M6"));
    EXPECT_GT(c.confidence, 0.5);
}

// ─── T5: ISO 273 table lookup is correct for several sizes ──────────────
TEST(SkillBoltHoleMetricSpec, ClearanceTableMatchesIso273)
{
    using skill::bolt_hole_metric_spec::clearanceDiameterFor;
    // Spot-check 3 sizes × 3 fits from the published ISO 273 table.
    EXPECT_NEAR(clearanceDiameterFor("M3",  "close"),   3.2, 1e-9);
    EXPECT_NEAR(clearanceDiameterFor("M3",  "medium"),  3.4, 1e-9);
    EXPECT_NEAR(clearanceDiameterFor("M3",  "free"),    3.6, 1e-9);
    EXPECT_NEAR(clearanceDiameterFor("M6",  "medium"),  6.6, 1e-9);
    EXPECT_NEAR(clearanceDiameterFor("M10", "free"),   12.0, 1e-9);
    EXPECT_NEAR(clearanceDiameterFor("M30", "close"),  31.0, 1e-9);
    EXPECT_DOUBLE_EQ(clearanceDiameterFor("M99", "medium"), 0.0);
    EXPECT_DOUBLE_EQ(clearanceDiameterFor("M6",  "weird"),  0.0);
}
