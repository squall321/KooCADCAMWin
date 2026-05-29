// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/unc_unf_hole_spec.hpp"

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

skill::unc_unf_hole_spec::Input goodInput()
{
    skill::unc_unf_hole_spec::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 25.0;
    in.position_y_mm     = 25.0;
    in.axis_dir          = gp_Dir(0, 0, -1);
    in.fastener_size     = "1/4-20";
    in.fit_class         = "normal";
    in.chamfer_size_mm   = 0.5;
    in.chamfer_angle_deg = 45.0;
    return in;
}

}  // namespace

// ─── T1: REAL volume — clearance cyl + chamfer, ±20% ────────────────────
TEST(SkillUncUnfHoleSpec, ApplyRemovesClearancePlusChamfer)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    auto out = skill::unc_unf_hole_spec::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double removed = volBefore - volumeOf(out.workpiece->shape());

    // 1/4-20 normal = 7.14 mm.  Through 10 mm.
    const double clrR    = 7.14 / 2.0;
    const double cylVol  = M_PI * clrR * clrR * 10.0;
    EXPECT_NEAR(removed, cylVol, cylVol * 0.20);
    EXPECT_GT(removed, cylVol);   // chamfer extra
}

// ─── T2: unknown fastener rejected ──────────────────────────────────────
TEST(SkillUncUnfHoleSpec, ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    auto in = goodInput();
    in.fastener_size = "M6";          // metric — not in imperial table
    auto r = skill::unc_unf_hole_spec::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-IMP-UNKNOWN"));
    EXPECT_THROW(skill::unc_unf_hole_spec::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.fit_class = "snug";
    auto r2 = skill::unc_unf_hole_spec::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-IMP-FIT"));
}

// ─── T3: signature carries spec ─────────────────────────────────────────
TEST(SkillUncUnfHoleSpec, SignatureCarriesSpecKey)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out   = skill::unc_unf_hole_spec::apply(*stock, goodInput());
    EXPECT_EQ(out.signature.pattern["kind"], std::string("unc_unf_hole_spec"));
    EXPECT_EQ(out.signature.pattern["is_compound"], true);
    EXPECT_EQ(out.signature.pattern["subfeature_count"], 2);
    EXPECT_EQ(out.signature.pattern["fastener_size"], std::string("1/4-20"));
    EXPECT_NEAR(out.signature.pattern["clearance_dia_mm"].get<double>(), 7.14, 1e-3);
}

// ─── T4: recognize finds the hole ───────────────────────────────────────
TEST(SkillUncUnfHoleSpec, RecognizeFindsMatchingClearance)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out   = skill::unc_unf_hole_spec::apply(*stock, goodInput());

    auto cands = skill::unc_unf_hole_spec::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().recovered_params["fastener_size"],
              std::string("1/4-20"));
}

// ─── T5: ASME B18.2.8 table values ──────────────────────────────────────
TEST(SkillUncUnfHoleSpec, AsmeTableValues)
{
    using skill::unc_unf_hole_spec::clearanceDiameterFor;
    EXPECT_NEAR(clearanceDiameterFor("#2-56",  "normal"),  2.39, 1e-3);
    EXPECT_NEAR(clearanceDiameterFor("#6-32",  "close"),   4.22, 1e-3);
    EXPECT_NEAR(clearanceDiameterFor("1/4-20", "normal"),  7.14, 1e-3);
    EXPECT_NEAR(clearanceDiameterFor("1/4-20", "loose"),   8.20, 1e-3);
    EXPECT_NEAR(clearanceDiameterFor("1/2-13", "close"),  14.27, 1e-3);
    EXPECT_DOUBLE_EQ(clearanceDiameterFor("M6",    "normal"), 0.0);
    EXPECT_DOUBLE_EQ(clearanceDiameterFor("1/4-20", "weird"), 0.0);
}
