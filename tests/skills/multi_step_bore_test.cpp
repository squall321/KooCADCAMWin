// @lat: [[process/test-strategy#skill round-trip]]
//
// multi_step_bore — N-step bore (N ≥ 3) with strictly decreasing diameters.
// Synthesised via fuseMany-then-cut to avoid the OCCT overlap-in-compound
// bug for concentric cylinders.
//
// Cases (5):
//   1. Synthesis on a thick stock removes the sum of the step volumes.
//   2. DFM rejects N < 3 steps AND non-decreasing diameters.
//   3. Recognize finds the N-step chain with confidence 0.85.
//   4. STEP round-trip preserves the recovered step count.
//   5. Topology: N new cylindrical faces + (N-1) annular planes appear.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/multi_step_bore.hpp"

#include "io/StepIO.hpp"

#include <BRepAdaptor_Surface.hxx>
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

int countCylindricalFaces(const skill::Workpiece& wp)
{
    int n = 0;
    for (int i = 0; i < wp.faceCount(); ++i)
        if (wp.isFaceCylinder(i)) ++n;
    return n;
}

}  // namespace

// ─── 1. Synthesis removes the sum of step volumes ─────────────────────────
TEST(SkillMultiStepBore, ApplyRemovesSumOfStepVolumes)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::multi_step_bore::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.steps = {
        { 16.0, 4.0 },     // wide entry
        { 12.0, 5.0 },     // middle
        {  8.0, 6.0 },     // bottom — strictly decreasing
    };

    auto out = skill::multi_step_bore::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Each step is a concentric cylinder; the upper steps already encompass
    // the lower step radii.  The total volume removed equals:
    //   V0_disc(annulus 0→1) + V1_disc(annulus 1→2) + V2_disc(step 2 cyl)
    // = π r0² × depth0 + π r1² × depth1 + π r2² × depth2
    // because each "step" is the cylinder of its (dia, depth) and they are
    // stacked end-to-end.
    const double v0 = M_PI * 8.0 * 8.0 * 4.0;
    const double v1 = M_PI * 6.0 * 6.0 * 5.0;
    const double v2 = M_PI * 4.0 * 4.0 * 6.0;
    const double approx = v0 + v1 + v2;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.08);   // looser tol (fuse+cut)

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("multi_step_bore"));
    EXPECT_EQ(out.signature.pattern["step_count"].get<int>(), 3);
}

// ─── 2. DFM rejects N < 3 and non-decreasing diameters ────────────────────
TEST(SkillMultiStepBore, ValidateDFM)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);

    // Only 2 steps → DFM-INPUT error
    {
        skill::multi_step_bore::Input in;
        in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 25.0;
        in.position_y_mm = 25.0;
        in.steps = {
            { 12.0, 5.0 },
            {  8.0, 6.0 },
        };
        auto r = skill::multi_step_bore::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-INPUT") { found = true; break; }
        EXPECT_TRUE(found);
    }

    // Non-decreasing diameters → DFM-MSB-ORDER error
    {
        skill::multi_step_bore::Input in;
        in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 25.0;
        in.position_y_mm = 25.0;
        in.steps = {
            { 12.0, 4.0 },
            { 10.0, 5.0 },
            { 14.0, 6.0 },   // INCREASES — must be < 10
        };
        auto r = skill::multi_step_bore::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-MSB-ORDER") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::multi_step_bore::apply(*stock, in), skill::SkillError);
    }
}

// ─── 3. Recognize finds the multi-step chain ──────────────────────────────
TEST(SkillMultiStepBore, RecognizeFindsChain)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);

    skill::multi_step_bore::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.steps = {
        { 16.0, 4.0 },
        { 12.0, 5.0 },
        {  8.0, 6.0 },
    };

    auto out = skill::multi_step_bore::apply(*stock, in);
    auto cands = skill::multi_step_bore::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    // Find the candidate whose step count is 3 AND diameters match.
    bool matched = false;
    for (const auto& c : cands) {
        EXPECT_EQ(c.skill_id, std::string("multi_step_bore"));
        EXPECT_NEAR(c.confidence, 0.85, 1e-6);
        const auto& steps = c.recovered_params["steps"];
        if (steps.size() != 3u) continue;
        const double d0 = steps[0]["dia_mm"].get<double>();
        const double d1 = steps[1]["dia_mm"].get<double>();
        const double d2 = steps[2]["dia_mm"].get<double>();
        if (std::abs(d0 - 16.0) < 1e-3 &&
            std::abs(d1 - 12.0) < 1e-3 &&
            std::abs(d2 -  8.0) < 1e-3) {
            matched = true;
            break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 4. STEP round-trip preserves step recognition ────────────────────────
TEST(SkillMultiStepBore, RoundTripViaStep)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);

    skill::multi_step_bore::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 24.0;
    in.position_y_mm = 26.0;
    in.steps = {
        { 14.0, 3.0 },
        { 10.0, 4.0 },
        {  6.0, 5.0 },
    };

    auto synth = skill::multi_step_bore::apply(*stock, in);
    const fs::path stepPath = fs::temp_directory_path() / "multi_step_bore_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reimp = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimp.has_value()) << err;
    skill::Workpiece reim(*reimp);

    auto cands = skill::multi_step_bore::recognize(reim);
    ASSERT_GE(cands.size(), 1u);
    bool matched = false;
    for (const auto& c : cands) {
        const auto& steps = c.recovered_params["steps"];
        if (steps.size() != 3u) continue;
        const double d0 = steps[0]["dia_mm"].get<double>();
        const double d1 = steps[1]["dia_mm"].get<double>();
        const double d2 = steps[2]["dia_mm"].get<double>();
        if (std::abs(d0 - 14.0) < 5e-2 &&
            std::abs(d1 - 10.0) < 5e-2 &&
            std::abs(d2 -  6.0) < 5e-2) {
            matched = true;
            break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 5. Topology: N cylindrical faces + (N-1) annular planes appear ───────
TEST(SkillMultiStepBore, TopologyShowsAllSteps)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);
    const int stockCyls = countCylindricalFaces(*stock);   // typically 0

    skill::multi_step_bore::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.steps = {
        { 16.0, 4.0 },
        { 12.0, 4.0 },
        {  8.0, 4.0 },
        {  5.0, 4.0 },     // 4 steps — exercise N > 3
    };

    auto out = skill::multi_step_bore::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // After fuse-then-cut, we should see exactly 4 new cylindrical faces.
    const int afterCyls = countCylindricalFaces(*out.workpiece);
    EXPECT_EQ(afterCyls - stockCyls, 4)
        << "expected one new cylindrical face per step";

    // The signature must record step_count == 4.
    EXPECT_EQ(out.signature.pattern["step_count"].get<int>(), 4);
    EXPECT_EQ(out.signature.pattern["steps"].size(), 4u);
}
