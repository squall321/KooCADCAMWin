// @lat: [[process/test-strategy#skill round-trip]]
//
// turn_external — outer-diameter turning on cylindrical stock.
//
// Cases (5):
//   1. apply removes the expected annular volume.
//   2. DFM rejects final_dia ≥ current_outer_dia (DFM-LATHE error).
//   3. recognize identifies the turned cylinder + shoulders.
//   4. STEP round-trip preserves recovered params.
//   5. DFM rejects end_z ≤ start_z (DFM-INPUT error).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/turn_external.hpp"

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

// ─── 1. Apply removes the annular volume ──────────────────────────────────
TEST(SkillTurnExternal, ApplyRemovesAnnularVolume)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);  // Ø40, length 20
    ASSERT_FALSE(stock->shape().IsNull());

    skill::turn_external::Input in;
    in.start_z_mm   = 5.0;
    in.end_z_mm     = 15.0;
    in.final_dia_mm = 30.0;       // reduce 40 → 30 over 10 mm
    in.roughing_passes = 2;

    auto out = skill::turn_external::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Approx volume removed = π·(R²−r²)·dz, R=20 r=15, dz=10
    const double approx = M_PI * (20.0 * 20.0 - 15.0 * 15.0) * 10.0;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.06);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("turn_external"));
}

// ─── 2. DFM rejects final_dia >= current outer dia ────────────────────────
TEST(SkillTurnExternal, ValidateRejectsTooLargeFinalDia)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::turn_external::Input in;
    in.start_z_mm   = 5.0;
    in.end_z_mm     = 15.0;
    in.final_dia_mm = 42.0;       // larger than current — invalid
    in.roughing_passes = 1;

    auto r = skill::turn_external::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LATHE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::turn_external::apply(*stock, in), skill::SkillError);
}

// ─── 3. Recognize finds the turned section ────────────────────────────────
TEST(SkillTurnExternal, RecognizeFindsTurnedSection)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::turn_external::Input in;
    in.start_z_mm   = 4.0;
    in.end_z_mm     = 16.0;
    in.final_dia_mm = 28.0;
    in.roughing_passes = 1;

    auto out = skill::turn_external::apply(*stock, in);
    auto cands = skill::turn_external::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    bool matched = false;
    for (const auto& c : cands) {
        if (std::abs(c.recovered_params["final_dia_mm"].get<double>() - in.final_dia_mm) < 0.1) {
            EXPECT_NEAR(c.recovered_params["start_z_mm"].get<double>(),
                        in.start_z_mm, 0.1);
            EXPECT_NEAR(c.recovered_params["end_z_mm"].get<double>(),
                        in.end_z_mm, 0.1);
            EXPECT_GT(c.confidence, 0.7);
            matched = true;
            break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 4. STEP round-trip preserves recognition ─────────────────────────────
TEST(SkillTurnExternal, RoundTripViaStep)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::turn_external::Input in;
    in.start_z_mm   = 5.0;
    in.end_z_mm     = 14.0;
    in.final_dia_mm = 32.0;
    in.roughing_passes = 1;

    auto synth = skill::turn_external::apply(*stock, in);

    const fs::path stepPath = fs::temp_directory_path() / "turn_external_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reimportedOpt = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimportedOpt.has_value()) << err;
    skill::Workpiece reim(*reimportedOpt);

    auto cands = skill::turn_external::recognize(reim);
    ASSERT_GE(cands.size(), 1u);

    bool matched = false;
    for (const auto& c : cands) {
        if (std::abs(c.recovered_params["final_dia_mm"].get<double>() - in.final_dia_mm) < 0.1) {
            matched = true;
            break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 5. DFM rejects end_z <= start_z ──────────────────────────────────────
TEST(SkillTurnExternal, ValidateRejectsInvertedZRange)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::turn_external::Input in;
    in.start_z_mm   = 15.0;
    in.end_z_mm     = 5.0;     // inverted
    in.final_dia_mm = 30.0;
    in.roughing_passes = 1;

    auto r = skill::turn_external::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);
}
