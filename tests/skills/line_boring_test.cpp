// @lat: [[process/test-strategy#skill round-trip]]
//
// line_boring — multi-coaxial bores through a series of bosses.  Slice-1
// produces a single through-cylinder spanning all bore_positions.
//
// Cases (5):
//   1. Synthesis on a tall stock removes a through-cylinder.
//   2. DFM rejects dia < 6 mm AND single-position input.
//   3. Recognize finds the long cylinder when span ≥ 2 × dia.
//   4. STEP round-trip preserves the recovered dia.
//   5. Multi-position input (3 bores, evenly spaced) produces a span
//      matching the input range + overhang.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/line_boring.hpp"

#include "io/StepIO.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
#include <filesystem>

namespace fs = std::filesystem;
using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Synthesis: cuts a single span cylinder across all bore positions ──
TEST(SkillLineBoring, ApplyCutsThroughCylinder)
{
    // Tall stock; in slice-1 line_boring produces a single cylinder spanning
    // min(bore_z)−overhang to max(bore_z)+overhang (not a 'through' hole).
    auto stock = skill::createCuboidStock(30.0, 30.0, 100.0);

    skill::line_boring::Input in;
    // Axis: from (15, 15, 0) along +Z (through the entire 100 mm length).
    in.axis              = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(15.0, 15.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.dia_mm            = 10.0;
    // 3 bores at Z = 20, 50, 80 (within the 100 mm length)
    in.bore_positions_z_mm = { 20.0, 50.0, 80.0 };
    in.segment_overhang_mm = 5.0;        // span = (80 - 20) + 2*5 = 70

    auto out = skill::line_boring::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Approx volume removed: π × 5² × 70 (the cut cylinder, intersected with
    // the 100 mm stock).  Note the cylinder may extend beyond the stock if
    // overhang pushes past Z = 100, so we use the actual intersection
    // length of 70 mm only when zLo ≥ 0 and zHi ≤ 100.  Here zLo = 15,
    // zHi = 85 — both inside the stock — so 70 mm is the right length.
    const double approx = M_PI * 25.0 * 70.0;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.05);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("line_boring"));
}

// ─── 2. DFM rejects sub-6mm dia and single-position input ─────────────────
TEST(SkillLineBoring, ValidateDFM)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 100.0);

    // dia 3 mm → DFM-LINEBORE-DIA error
    {
        skill::line_boring::Input in;
        in.axis = skill::FaceCylinderByAxis{
            gp_Ax1(gp_Pnt(15.0, 15.0, 0.0), gp_Dir(0, 0, 1)), 5.0
        };
        in.dia_mm            = 3.0;
        in.bore_positions_z_mm = { 20.0, 80.0 };
        auto r = skill::line_boring::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-LINEBORE-DIA") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::line_boring::apply(*stock, in), skill::SkillError);
    }

    // single position → DFM-LINEBORE-COUNT error
    {
        skill::line_boring::Input in;
        in.axis = skill::FaceCylinderByAxis{
            gp_Ax1(gp_Pnt(15.0, 15.0, 0.0), gp_Dir(0, 0, 1)), 5.0
        };
        in.dia_mm            = 10.0;
        in.bore_positions_z_mm = { 50.0 };       // only 1 — degenerate
        auto r = skill::line_boring::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-LINEBORE-COUNT") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ─── 3. Recognize finds the line-bore on a tall stock ─────────────────────
TEST(SkillLineBoring, RecognizeFindsLongCylinder)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 100.0);

    skill::line_boring::Input in;
    in.axis              = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(15.0, 15.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.dia_mm            = 10.0;
    in.bore_positions_z_mm = { 20.0, 50.0, 80.0 };
    in.segment_overhang_mm = 5.0;

    auto out = skill::line_boring::apply(*stock, in);
    auto cands = skill::line_boring::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    // Confidence 0.5 — ambiguous with through_hole in slice-1.
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("line_boring"));
    EXPECT_NEAR(c.confidence, 0.5, 1e-6);
    EXPECT_NEAR(c.recovered_params["dia_mm"].get<double>(), 10.0, 1e-3);
    EXPECT_GE(c.matched_geometry["span_axial_mm"].get<double>(), 2.0 * 10.0);
}

// ─── 4. STEP round-trip preserves the recovered diameter ──────────────────
TEST(SkillLineBoring, RoundTripViaStep)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 120.0);

    skill::line_boring::Input in;
    in.axis              = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(20.0, 20.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.dia_mm            = 12.0;
    in.bore_positions_z_mm = { 25.0, 60.0, 95.0 };
    in.segment_overhang_mm = 4.0;

    auto synth = skill::line_boring::apply(*stock, in);
    const fs::path stepPath = fs::temp_directory_path() / "line_boring_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reimp = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimp.has_value()) << err;
    skill::Workpiece reim(*reimp);

    auto cands = skill::line_boring::recognize(reim);
    ASSERT_GE(cands.size(), 1u);
    bool matched = false;
    for (const auto& c : cands) {
        if (std::abs(c.recovered_params["dia_mm"].get<double>() - in.dia_mm) < 5e-2) {
            matched = true;
            break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 5. Multi-position span = max - min + 2 × overhang ────────────────────
TEST(SkillLineBoring, MultiPositionSpanCorrect)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 100.0);

    skill::line_boring::Input in;
    in.axis              = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(15.0, 15.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.dia_mm            = 8.0;
    in.bore_positions_z_mm = { 10.0, 40.0, 70.0 };   // span = 70 - 10 = 60
    in.segment_overhang_mm = 3.0;                    // total = 60 + 6 = 66

    auto out = skill::line_boring::apply(*stock, in);

    // Verify the pattern.span_axial_mm in the signature equals 66.
    const auto& sig = out.signature;
    ASSERT_TRUE(sig.pattern.contains("span_axial_mm"));
    EXPECT_NEAR(sig.pattern["span_axial_mm"].get<double>(), 66.0, 1e-6);
    EXPECT_EQ(sig.pattern["bore_positions"].size(), 3u);
}
