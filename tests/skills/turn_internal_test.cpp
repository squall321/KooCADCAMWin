// @lat: [[process/test-strategy#skill round-trip]]
//
// turn_internal — internal bore enlargement on cylindrical stock.
//
// Cases (5):
//   1. apply (pre-existing bore) removes annular volume = π(R²-r²)·dz.
//   2. DFM rejects final_dia <= existing_bore_dia (DFM-INPUT).
//   3. recognize identifies the enlarged section + step shoulder.
//   4. STEP round-trip preserves recognition.
//   5. DFM rejects diameters < 0.8 mm (DFM-002).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
#include "skills/turn_internal.hpp"

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

// Pre-drill a starter bore down the Z axis of the cylindrical stock so that
// turn_internal has something to enlarge.
std::shared_ptr<skill::Workpiece> stockWithStarterBore(double od_mm, double len_mm,
                                                       double bore_dia_mm,
                                                       double bore_depth_mm)
{
    auto stock = skill::createCylindricalStock(od_mm, len_mm);
    skill::drill_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 0.0;
    in.position_y_mm = 0.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.diameter_mm   = bore_dia_mm;
    in.depth_mm      = bore_depth_mm;
    in.through_hole  = false;
    auto out = skill::drill_hole::apply(*stock, in);
    return out.workpiece;
}

}  // namespace

// ─── 1. Apply enlarges bore — annular volume removed ──────────────────────
TEST(SkillTurnInternal, ApplyEnlargesBore)
{
    // Ø40 stock, length 20, pre-bore Ø10 × 15 deep (starting at top Z=20).
    auto wp = stockWithStarterBore(40.0, 20.0, 10.0, 15.0);
    const double volBefore = volumeOf(wp->shape());

    skill::turn_internal::Input in;
    in.start_z_mm           = 8.0;
    in.end_z_mm             = 18.0;   // within the existing bore (Z range 5..20)
    in.final_dia_mm         = 16.0;
    in.existing_bore_dia_mm = 10.0;

    auto out = skill::turn_internal::apply(*wp, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    // Material removed = π(R²-r²)·dz, R=8 r=5 dz=10 ≈ 122.5π
    const double approx = M_PI * (8.0 * 8.0 - 5.0 * 5.0) * 10.0;
    EXPECT_NEAR(volBefore - volAfter, approx, approx * 0.10);

    EXPECT_EQ(out.signature.skill_id, std::string("turn_internal"));
}

// ─── 2. DFM rejects final_dia <= existing_bore_dia ────────────────────────
TEST(SkillTurnInternal, ValidateRejectsNonEnlargement)
{
    auto wp = stockWithStarterBore(40.0, 20.0, 10.0, 15.0);

    skill::turn_internal::Input in;
    in.start_z_mm           = 8.0;
    in.end_z_mm             = 18.0;
    in.final_dia_mm         = 10.0;     // equal to existing — not an enlargement
    in.existing_bore_dia_mm = 10.0;

    auto r = skill::turn_internal::validate(*wp, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT" && f.message.find("must be >") != std::string::npos) {
            found = true; break;
        }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::turn_internal::apply(*wp, in), skill::SkillError);
}

// ─── 3. Recognize finds the enlarged section + step ───────────────────────
TEST(SkillTurnInternal, RecognizeFindsEnlargement)
{
    auto wp = stockWithStarterBore(40.0, 20.0, 10.0, 15.0);

    skill::turn_internal::Input in;
    in.start_z_mm           = 8.0;
    in.end_z_mm             = 18.0;
    in.final_dia_mm         = 16.0;
    in.existing_bore_dia_mm = 10.0;

    auto out = skill::turn_internal::apply(*wp, in);
    auto cands = skill::turn_internal::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    bool matched = false;
    for (const auto& c : cands) {
        const double fd = c.recovered_params["final_dia_mm"].get<double>();
        const double ed = c.recovered_params["existing_bore_dia_mm"].get<double>();
        if (std::abs(fd - in.final_dia_mm) < 0.1 &&
            std::abs(ed - in.existing_bore_dia_mm) < 0.1) {
            EXPECT_GT(c.confidence, 0.7);
            matched = true;
            break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 4. STEP round-trip preserves recognition ─────────────────────────────
TEST(SkillTurnInternal, RoundTripViaStep)
{
    auto wp = stockWithStarterBore(40.0, 20.0, 10.0, 15.0);

    skill::turn_internal::Input in;
    in.start_z_mm           = 8.0;
    in.end_z_mm             = 18.0;
    in.final_dia_mm         = 18.0;
    in.existing_bore_dia_mm = 10.0;

    auto synth = skill::turn_internal::apply(*wp, in);

    const fs::path stepPath = fs::temp_directory_path() / "turn_internal_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reimportedOpt = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimportedOpt.has_value()) << err;
    skill::Workpiece reim(*reimportedOpt);

    auto cands = skill::turn_internal::recognize(reim);
    ASSERT_GE(cands.size(), 1u);

    bool matched = false;
    for (const auto& c : cands) {
        if (std::abs(c.recovered_params["final_dia_mm"].get<double>() - in.final_dia_mm) < 0.1 &&
            std::abs(c.recovered_params["existing_bore_dia_mm"].get<double>() - in.existing_bore_dia_mm) < 0.1) {
            matched = true; break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 5. DFM rejects sub-0.8 mm diameter ───────────────────────────────────
TEST(SkillTurnInternal, ValidateRejectsSubMinDia)
{
    auto wp = stockWithStarterBore(40.0, 20.0, 10.0, 15.0);

    skill::turn_internal::Input in;
    in.start_z_mm           = 8.0;
    in.end_z_mm             = 18.0;
    in.final_dia_mm         = 0.5;       // sub-min
    in.existing_bore_dia_mm = 0.3;       // sub-min

    auto r = skill::turn_internal::validate(*wp, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { found = true; break; }
    EXPECT_TRUE(found);
}
