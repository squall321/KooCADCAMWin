// @lat: [[process/test-strategy#skill round-trip]]
//
// groove_turn — annular external groove on cylindrical stock.
//
// Cases (5):
//   1. apply removes the annular groove volume.
//   2. DFM rejects width < 0.5 mm (DFM-INPUT).
//   3. recognize identifies groove cylinder + 2 walls.
//   4. STEP round-trip preserves recognition.
//   5. DFM rejects depth too large (DFM-INPUT).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/groove_turn.hpp"

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

// ─── 1. Apply removes the annular groove volume ───────────────────────────
TEST(SkillGrooveTurn, ApplyRemovesGroove)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);   // Ø40, length 30
    ASSERT_FALSE(stock->shape().IsNull());

    skill::groove_turn::Input in;
    in.center_z_mm = 15.0;
    in.width_mm    = 4.0;
    in.depth_mm    = 5.0;

    auto out = skill::groove_turn::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume removed = π·(R²-r²)·width, R=20 r=15 width=4 → π·175·4 = 700π
    const double approx = M_PI * (20.0 * 20.0 - 15.0 * 15.0) * 4.0;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.06);

    EXPECT_EQ(out.signature.skill_id, std::string("groove_turn"));
}

// ─── 2. DFM rejects width < 0.5 ───────────────────────────────────────────
TEST(SkillGrooveTurn, ValidateRejectsTooThinWidth)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::groove_turn::Input in;
    in.center_z_mm = 15.0;
    in.width_mm    = 0.2;       // < 0.5
    in.depth_mm    = 3.0;

    auto r = skill::groove_turn::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT" && f.message.find("0.5") != std::string::npos) {
            found = true; break;
        }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::groove_turn::apply(*stock, in), skill::SkillError);
}

// ─── 3. Recognize finds the groove ────────────────────────────────────────
TEST(SkillGrooveTurn, RecognizeFindsGroove)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::groove_turn::Input in;
    in.center_z_mm = 12.0;
    in.width_mm    = 5.0;
    in.depth_mm    = 4.0;

    auto out = skill::groove_turn::apply(*stock, in);
    auto cands = skill::groove_turn::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    bool matched = false;
    for (const auto& c : cands) {
        if (std::abs(c.recovered_params["center_z_mm"].get<double>() - in.center_z_mm) < 0.1 &&
            std::abs(c.recovered_params["width_mm"].get<double>()    - in.width_mm)    < 0.1 &&
            std::abs(c.recovered_params["depth_mm"].get<double>()    - in.depth_mm)    < 0.1) {
            EXPECT_GT(c.confidence, 0.7);
            matched = true; break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 4. STEP round-trip preserves recognition ─────────────────────────────
TEST(SkillGrooveTurn, RoundTripViaStep)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::groove_turn::Input in;
    in.center_z_mm = 18.0;
    in.width_mm    = 3.0;
    in.depth_mm    = 6.0;

    auto synth = skill::groove_turn::apply(*stock, in);

    const fs::path stepPath = fs::temp_directory_path() / "groove_turn_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reimportedOpt = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimportedOpt.has_value()) << err;
    skill::Workpiece reim(*reimportedOpt);

    auto cands = skill::groove_turn::recognize(reim);
    ASSERT_GE(cands.size(), 1u);

    bool matched = false;
    for (const auto& c : cands) {
        if (std::abs(c.recovered_params["center_z_mm"].get<double>() - in.center_z_mm) < 0.1 &&
            std::abs(c.recovered_params["width_mm"].get<double>()    - in.width_mm)    < 0.1) {
            matched = true; break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 5. DFM rejects too deep ──────────────────────────────────────────────
TEST(SkillGrooveTurn, ValidateRejectsTooDeep)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);   // outerR = 20

    skill::groove_turn::Input in;
    in.center_z_mm = 15.0;
    in.width_mm    = 3.0;
    in.depth_mm    = 19.5;     // ≥ outerR-1 = 19 → rejected

    auto r = skill::groove_turn::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}
