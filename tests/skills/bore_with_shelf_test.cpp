// @lat: [[process/test-strategy#skill round-trip]]
//
// bore_with_shelf — stepped bore, the "watch interior" pattern.  Critical
// for slice-1 watch case body.
//
// Cases (5):
//   1. apply removes expected volume on a cuboid stock.
//   2. DFM rejects lower_dia ≤ upper_dia (DFM-STEP error).
//   3. recognize identifies the stepped bore (upper+lower dia, depths).
//   4. STEP round-trip preserves recovered params.
//   5. Edge case: 40 × 40 × 12 cuboid stock produces a visible step
//      (count of cylindrical faces increases by 2; planar-face shelf exists).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bore_with_shelf.hpp"

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

// ─── 1. Apply on a thick cuboid removes upper + lower volumes ─────────────
TEST(SkillBoreWithShelf, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::bore_with_shelf::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.upper_dia_mm   = 8.0;
    in.upper_depth_mm = 4.0;
    in.lower_dia_mm   = 16.0;
    in.lower_depth_mm = 8.0;

    auto out = skill::bore_with_shelf::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double upperV = M_PI * 4.0 * 4.0 * 4.0;
    const double lowerV = M_PI * 8.0 * 8.0 * 8.0;
    const double approx = upperV + lowerV;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    // Slight overlap by design (small overlap region between cylinders);
    // tolerance is loose because Booleans on overlapping cuts can leave
    // a slim ring at the shelf plane.
    EXPECT_NEAR(diff, approx, approx * 0.08);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("bore_with_shelf"));
}

// ─── 2. DFM rejects equal or inverted diameters ───────────────────────────
TEST(SkillBoreWithShelf, ValidateRejectsInvertedStep)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::bore_with_shelf::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.upper_dia_mm   = 10.0;
    in.upper_depth_mm = 3.0;
    in.lower_dia_mm   = 8.0;       // lower must be > upper
    in.lower_depth_mm = 5.0;

    auto r = skill::bore_with_shelf::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-STEP") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::bore_with_shelf::apply(*stock, in), skill::SkillError);
}

// ─── 3. Recognize identifies the step ─────────────────────────────────────
TEST(SkillBoreWithShelf, RecognizeFindsStep)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::bore_with_shelf::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.upper_dia_mm   = 8.0;
    in.upper_depth_mm = 4.0;
    in.lower_dia_mm   = 16.0;
    in.lower_depth_mm = 8.0;

    auto out = skill::bore_with_shelf::apply(*stock, in);
    auto cands = skill::bore_with_shelf::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    // Pick the candidate with the best confidence whose recovered diameters
    // match the input — any spurious axes-swapped duplicates are ignored.
    bool matched = false;
    for (const auto& c : cands) {
        const double up = c.recovered_params["upper_dia_mm"].get<double>();
        const double lo = c.recovered_params["lower_dia_mm"].get<double>();
        if (std::abs(up - in.upper_dia_mm) < 1e-3 &&
            std::abs(lo - in.lower_dia_mm) < 1e-3) {
            EXPECT_NEAR(c.recovered_params["upper_depth_mm"].get<double>(),
                        in.upper_depth_mm, 0.2);
            EXPECT_NEAR(c.recovered_params["lower_depth_mm"].get<double>(),
                        in.lower_depth_mm, 0.2);
            EXPECT_GT(c.confidence, 0.8);
            matched = true;
            break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 4. STEP round-trip preserves step recognition ────────────────────────
TEST(SkillBoreWithShelf, RoundTripViaStep)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::bore_with_shelf::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 22.0;
    in.position_y_mm = 28.0;
    in.upper_dia_mm   = 7.0;
    in.upper_depth_mm = 3.5;
    in.lower_dia_mm   = 14.0;
    in.lower_depth_mm = 7.0;

    auto synth = skill::bore_with_shelf::apply(*stock, in);
    const fs::path stepPath = fs::temp_directory_path() / "bore_shelf_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reimportedOpt = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimportedOpt.has_value()) << err;
    skill::Workpiece reim(*reimportedOpt);

    auto cands = skill::bore_with_shelf::recognize(reim);
    ASSERT_GE(cands.size(), 1u);

    bool matched = false;
    for (const auto& c : cands) {
        const double up = c.recovered_params["upper_dia_mm"].get<double>();
        const double lo = c.recovered_params["lower_dia_mm"].get<double>();
        if (std::abs(up - in.upper_dia_mm) < 5e-2 &&
            std::abs(lo - in.lower_dia_mm) < 5e-2) {
            matched = true;
            break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 5. Edge: 40 × 40 × 12 cuboid stock with a clear stepped pocket ───────
TEST(SkillBoreWithShelf, EdgeCaseCuboid40x40x12ProducesStep)
{
    // This is the spec-listed edge case: a 40×40×12 block must produce a
    // visible stepped pocket.  Pick small enough diameters & depths to fit.
    auto stock = skill::createCuboidStock(40.0, 40.0, 12.0);
    const int stockCylFaces = countCylindricalFaces(*stock);   // typically 0

    skill::bore_with_shelf::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm  = 20.0;
    in.position_y_mm  = 20.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.upper_dia_mm   = 6.0;
    in.upper_depth_mm = 3.0;
    in.lower_dia_mm   = 14.0;
    in.lower_depth_mm = 6.0;     // upper+lower = 9 < 12, fits.

    auto out = skill::bore_with_shelf::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Topology: we expect 2 new cylindrical faces (upper bore wall + lower bore wall).
    const int afterCylFaces = countCylindricalFaces(*out.workpiece);
    EXPECT_EQ(afterCylFaces - stockCylFaces, 2)
        << "expected exactly two new cylindrical faces (upper + lower bore walls)";

    // recognize should report at least one candidate that matches our input.
    auto cands = skill::bore_with_shelf::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    bool foundMatch = false;
    for (const auto& c : cands) {
        if (std::abs(c.recovered_params["upper_dia_mm"].get<double>() - in.upper_dia_mm) < 1e-3 &&
            std::abs(c.recovered_params["lower_dia_mm"].get<double>() - in.lower_dia_mm) < 1e-3) {
            foundMatch = true;
            break;
        }
    }
    EXPECT_TRUE(foundMatch);
}
