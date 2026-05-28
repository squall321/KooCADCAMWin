// @lat: [[process/test-strategy#skill round-trip]]
//
// face_turn — squaring the end face of a cylindrical part.
//
// Cases (5):
//   1. apply removes the slab between face_z and zMax.
//   2. DFM rejects face_z >= zMax (DFM-INPUT).
//   3. recognize identifies the squared top face.
//   4. STEP round-trip preserves recognition.
//   5. DFM rejects face_z <= zMin (DFM-INPUT — would destroy part).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/face_turn.hpp"

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

// ─── 1. Apply removes the top slab ────────────────────────────────────────
TEST(SkillFaceTurn, ApplyRemovesTopSlab)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);   // Ø40, length 20
    ASSERT_FALSE(stock->shape().IsNull());

    skill::face_turn::Input in;
    in.face_z_mm      = 18.0;             // bring top from 20 → 18 (removes 2 mm slab)
    in.surface_finish = "ra_0.8";

    auto out = skill::face_turn::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Removed volume ≈ π·R²·dh = π·20²·2 = 800π
    const double approx = M_PI * 20.0 * 20.0 * 2.0;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("face_turn"));
}

// ─── 2. DFM rejects face_z >= zMax ────────────────────────────────────────
TEST(SkillFaceTurn, ValidateRejectsFaceAtOrAboveTop)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::face_turn::Input in;
    in.face_z_mm      = 20.0;       // equals zMax — nothing to remove
    in.surface_finish = "ra_1.6";

    auto r = skill::face_turn::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::face_turn::apply(*stock, in), skill::SkillError);
}

// ─── 3. Recognize finds the squared top face ──────────────────────────────
TEST(SkillFaceTurn, RecognizeFindsSquaredFace)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::face_turn::Input in;
    in.face_z_mm      = 17.0;
    in.surface_finish = "ra_0.8";

    auto out = skill::face_turn::apply(*stock, in);
    auto cands = skill::face_turn::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    bool matched = false;
    for (const auto& c : cands) {
        if (std::abs(c.recovered_params["face_z_mm"].get<double>() - 17.0) < 0.1) {
            EXPECT_GT(c.confidence, 0.7);
            matched = true; break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 4. STEP round-trip preserves recognition ─────────────────────────────
TEST(SkillFaceTurn, RoundTripViaStep)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::face_turn::Input in;
    in.face_z_mm      = 15.0;
    in.surface_finish = "ra_0.4";

    auto synth = skill::face_turn::apply(*stock, in);

    const fs::path stepPath = fs::temp_directory_path() / "face_turn_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reimportedOpt = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimportedOpt.has_value()) << err;
    skill::Workpiece reim(*reimportedOpt);

    auto cands = skill::face_turn::recognize(reim);
    ASSERT_GE(cands.size(), 1u);

    bool matched = false;
    for (const auto& c : cands) {
        if (std::abs(c.recovered_params["face_z_mm"].get<double>() - in.face_z_mm) < 0.1) {
            matched = true; break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 5. DFM rejects face_z <= zMin ────────────────────────────────────────
TEST(SkillFaceTurn, ValidateRejectsFaceAtOrBelowBottom)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::face_turn::Input in;
    in.face_z_mm      = -1.0;     // below zMin (=0) — would destroy part
    in.surface_finish = "ra_1.6";

    auto r = skill::face_turn::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT" && f.message.find("zMin") != std::string::npos) {
            found = true; break;
        }
    EXPECT_TRUE(found);
}
