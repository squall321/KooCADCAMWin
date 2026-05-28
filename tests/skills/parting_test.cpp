// @lat: [[process/test-strategy#skill round-trip]]
//
// parting — sever the cylindrical workpiece at a given Z.  Result is a
// TopoDS_Compound containing two solid sub-shapes.
//
// Cases (5):
//   1. apply produces ≥ 2 solid sub-shapes (compound) and removes kerf vol.
//   2. DFM rejects cut_width < 1.0 (parting blade min).
//   3. recognize identifies the two cut planes.
//   4. STEP round-trip preserves recognition.
//   5. DFM rejects cut_z outside the bbox Z range.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/parting.hpp"

#include "io/StepIO.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
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

int countSolids(const TopoDS_Shape& s)
{
    int n = 0;
    for (TopExp_Explorer e(s, TopAbs_SOLID); e.More(); e.Next()) ++n;
    return n;
}

}  // namespace

// ─── 1. Apply produces 2 solids and removes the kerf volume ───────────────
TEST(SkillParting, ApplyProducesTwoSolids)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);   // Ø40, length 30
    ASSERT_FALSE(stock->shape().IsNull());
    ASSERT_EQ(countSolids(stock->shape()), 1);

    skill::parting::Input in;
    in.cut_z_mm     = 15.0;
    in.cut_width_mm = 1.5;

    auto out = skill::parting::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GE(countSolids(out.workpiece->shape()), 2)
        << "parting should split workpiece into ≥ 2 solids";

    // Volume removed = π·R²·width ≈ π·400·1.5 = 600π
    const double approx = M_PI * 20.0 * 20.0 * 1.5;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.07);

    EXPECT_EQ(out.signature.skill_id, std::string("parting"));
}

// ─── 2. DFM rejects too-thin parting blade ────────────────────────────────
TEST(SkillParting, ValidateRejectsThinBlade)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::parting::Input in;
    in.cut_z_mm     = 15.0;
    in.cut_width_mm = 0.5;   // < 1.0

    auto r = skill::parting::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT" && f.message.find("1.0") != std::string::npos) {
            found = true; break;
        }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::parting::apply(*stock, in), skill::SkillError);
}

// ─── 3. Recognize finds the two cut surfaces ──────────────────────────────
// `Workpiece::enumerate()` (using TopExp::MapShapes recursively) already
// walks all sub-shapes of a Compound — so the parted workpiece's 6 faces
// from both sub-solids are visible to wp.faceCount().  The actual bug was
// that parting::recognize() used `gp_Pln::Axis::Direction()` (surface
// normal — orientation-agnostic) instead of `wp.faceNormal()` (which
// respects TopAbs_REVERSED face flips).  After cutting, the upper solid's
// bottom face is a REVERSED face on a +Z surface plane; the geometric
// outward normal is -Z, which the orientation-aware accessor now reports
// correctly.
TEST(SkillParting, RecognizeFindsCutPlanes)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::parting::Input in;
    in.cut_z_mm     = 18.0;
    in.cut_width_mm = 2.0;

    auto out = skill::parting::apply(*stock, in);
    auto cands = skill::parting::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    bool matched = false;
    for (const auto& c : cands) {
        if (std::abs(c.recovered_params["cut_z_mm"].get<double>()     - in.cut_z_mm)     < 0.1 &&
            std::abs(c.recovered_params["cut_width_mm"].get<double>() - in.cut_width_mm) < 0.1) {
            EXPECT_GT(c.confidence, 0.7);
            matched = true; break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 4. STEP round-trip preserves recognition ─────────────────────────────
TEST(SkillParting, RoundTripViaStep)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::parting::Input in;
    in.cut_z_mm     = 12.0;
    in.cut_width_mm = 1.5;

    auto synth = skill::parting::apply(*stock, in);

    const fs::path stepPath = fs::temp_directory_path() / "parting_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reimportedOpt = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimportedOpt.has_value()) << err;
    skill::Workpiece reim(*reimportedOpt);

    auto cands = skill::parting::recognize(reim);
    ASSERT_GE(cands.size(), 1u);

    bool matched = false;
    for (const auto& c : cands) {
        if (std::abs(c.recovered_params["cut_z_mm"].get<double>()     - in.cut_z_mm)     < 0.2 &&
            std::abs(c.recovered_params["cut_width_mm"].get<double>() - in.cut_width_mm) < 0.2) {
            matched = true; break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 5. DFM rejects cut_z outside bbox ────────────────────────────────────
TEST(SkillParting, ValidateRejectsCutOutsideBbox)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);   // zMin=0, zMax=30

    skill::parting::Input in;
    in.cut_z_mm     = 40.0;     // above zMax
    in.cut_width_mm = 1.5;

    auto r = skill::parting::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);
}
