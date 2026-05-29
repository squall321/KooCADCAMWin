// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/threaded_through_with_chamfers.hpp"

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

skill::threaded_through_with_chamfers::Input goodInput()
{
    skill::threaded_through_with_chamfers::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.axis_dir         = gp_Dir(0, 0, -1);
    in.thread_size      = "M6";
    in.chamfer_size_mm  = 0.5;
    in.add_relief       = true;
    in.relief_offset_mm = 2.0;
    return in;
}

}  // namespace

// ─── T1: 4-cut compound removes pilot column + extras ───────────────────
TEST(SkillThreadedThroughWithChamfers, ApplyRemovesFourCutCompound)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    auto out = skill::threaded_through_with_chamfers::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double removed = volBefore - volumeOf(out.workpiece->shape());

    // M6 pilot = 5.0 mm.  Through 10 mm.
    const double pilotR = 5.0 / 2.0;
    const double cylVol = M_PI * pilotR * pilotR * 10.0;
    EXPECT_NEAR(removed, cylVol, cylVol * 0.20);
    EXPECT_GT(removed, cylVol);   // entry + exit chamfer + relief all add
}

// ─── T2: unknown thread rejected ────────────────────────────────────────
TEST(SkillThreadedThroughWithChamfers, ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    auto in = goodInput();
    in.thread_size = "M0.5";
    auto r = skill::threaded_through_with_chamfers::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-METRIC-UNKNOWN"));
    EXPECT_THROW(skill::threaded_through_with_chamfers::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3: signature carries spec + is_compound + 4 subfeatures ──────────
TEST(SkillThreadedThroughWithChamfers, SignatureCarriesSpec)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out   = skill::threaded_through_with_chamfers::apply(*stock, goodInput());
    EXPECT_EQ(out.signature.pattern["kind"],
              std::string("threaded_through_with_chamfers"));
    EXPECT_EQ(out.signature.pattern["is_compound"], true);
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 3);
    EXPECT_EQ(out.signature.pattern["thread_size"], std::string("M6"));
    EXPECT_NEAR(out.signature.pattern["pilot_dia_mm"].get<double>(),  5.0, 1e-3);
    EXPECT_NEAR(out.signature.pattern["pitch_mm"].get<double>(),      1.0, 1e-3);
    // relief_dia = pilot + 2*pitch = 5 + 2 = 7.
    EXPECT_NEAR(out.signature.pattern["relief_dia_mm"].get<double>(), 7.0, 1e-3);
}

// ─── T4: recognize finds the matching pilot ─────────────────────────────
TEST(SkillThreadedThroughWithChamfers, RecognizeFindsThreadSize)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out   = skill::threaded_through_with_chamfers::apply(*stock, goodInput());

    auto cands = skill::threaded_through_with_chamfers::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().recovered_params["thread_size"], std::string("M6"));
}

// ─── T5: spec-table values & relief geom rule ───────────────────────────
TEST(SkillThreadedThroughWithChamfers, ReliefGeomFollowsPitchSpec)
{
    using skill::threaded_through_with_chamfers::pilotDiameterFor;
    EXPECT_NEAR(pilotDiameterFor("M6"),  5.00, 1e-9);
    EXPECT_NEAR(pilotDiameterFor("M8"),  6.80, 1e-9);
    EXPECT_NEAR(pilotDiameterFor("M10"), 8.50, 1e-9);

    // Synthesize an M8 (pitch 1.25) and verify relief geometry:
    // relief_dia = 6.8 + 2*1.25 = 9.3
    auto stock = skill::createCuboidStock(50.0, 50.0, 12.0);
    auto in = goodInput();
    in.thread_size = "M8";
    auto out = skill::threaded_through_with_chamfers::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["relief_dia_mm"].get<double>(),   9.3,  1e-3);
    EXPECT_NEAR(out.signature.pattern["relief_width_mm"].get<double>(), 1.25, 1e-3);
}
