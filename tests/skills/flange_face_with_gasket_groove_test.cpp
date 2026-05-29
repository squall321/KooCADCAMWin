// @lat: [[process/test-strategy#skill round-trip]]
//
// flange_face_with_gasket_groove — REAL geometric verification.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/flange_face_with_gasket_groove.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

skill::flange_face_with_gasket_groove::Input goodInput()
{
    skill::flange_face_with_gasket_groove::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm           = 100.0;
    in.center_y_mm           = 100.0;
    in.axis_dir              = gp_Dir(0, 0, -1);
    in.flange_outer_dia_mm   = 180.0;
    in.facing_pass_depth_mm  = 0.5;
    in.gasket_groove_id_mm   = 120.0;
    in.gasket_groove_od_mm   = 130.0;
    in.gasket_groove_depth_mm = 1.5;
    in.groove_fillet_r_mm    = 0.4;
    return in;
}

}  // namespace

// ─── 1. Apply produces REAL volume delta matching expected geometry ───────
TEST(SkillFlangeFaceWithGasketGroove, ApplyRemovesExpectedVolume)
{
    // Cylindrical flange blank: dia 200, thick 30 → big enough.
    auto stock = skill::createCuboidStock(200.0, 200.0, 30.0);
    ASSERT_FALSE(stock->shape().IsNull());

    const double volBefore = volumeOf(stock->shape());

    auto in  = goodInput();
    auto out = skill::flange_face_with_gasket_groove::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected REMOVED = facing pass + gasket groove.
    const double Router = in.flange_outer_dia_mm / 2.0;
    const double Rod    = in.gasket_groove_od_mm / 2.0;
    const double Rid    = in.gasket_groove_id_mm / 2.0;
    const double vFacing = M_PI * Router * Router * in.facing_pass_depth_mm;
    const double vGroove = M_PI * (Rod * Rod - Rid * Rid) *
                           in.gasket_groove_depth_mm;
    const double expectedRemoved = vFacing + vGroove;

    const double actualRemoved = volBefore - volumeOf(out.workpiece->shape());
    EXPECT_GT(actualRemoved, 0.0);
    EXPECT_NEAR(actualRemoved, expectedRemoved, expectedRemoved * 0.20)
        << "actual=" << actualRemoved << " expected≈" << expectedRemoved;
}

// ─── 2. Validate rejects too-small fillet + apply throws ──────────────────
TEST(SkillFlangeFaceWithGasketGroove, ValidateRejectsSharpFillet)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 30.0);
    auto in    = goodInput();
    in.groove_fillet_r_mm = 0.1;   // < 0.4 mm

    auto r = skill::flange_face_with_gasket_groove::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-ASME-B16.20"));
    EXPECT_THROW(skill::flange_face_with_gasket_groove::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature is_compound=true & subfeature_count == 2 ────────────────
TEST(SkillFlangeFaceWithGasketGroove, SignatureCompound)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::flange_face_with_gasket_groove::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("flange_face_with_gasket_groove"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 2);

    // Params round-trip.
    EXPECT_NEAR(
        out.signature.params.value("gasket_groove_id_mm", 0.0),
        in.gasket_groove_id_mm, 1e-6);
    EXPECT_NEAR(
        out.signature.params.value("gasket_groove_od_mm", 0.0),
        in.gasket_groove_od_mm, 1e-6);
}

// ─── 4. Recognize finds a candidate with confidence > 0.5 ─────────────────
TEST(SkillFlangeFaceWithGasketGroove, RecognizeFindsGroove)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::flange_face_with_gasket_groove::apply(*stock, in);

    auto cands = skill::flange_face_with_gasket_groove::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    bool ok = false;
    for (const auto& c : cands) {
        if (c.confidence > 0.5) { ok = true; break; }
    }
    EXPECT_TRUE(ok);
}

// ─── 5. Feature-specific: gasket groove ID/OD recovered ───────────────────
TEST(SkillFlangeFaceWithGasketGroove, RecoveredGrooveDimensions)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 30.0);
    auto in    = goodInput();
    in.gasket_groove_id_mm = 110.0;
    in.gasket_groove_od_mm = 124.0;
    auto out   = skill::flange_face_with_gasket_groove::apply(*stock, in);

    auto cands = skill::flange_face_with_gasket_groove::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found = false;
    for (const auto& c : cands) {
        const double idRec = c.recovered_params.value("gasket_groove_id_mm", 0.0);
        const double odRec = c.recovered_params.value("gasket_groove_od_mm", 0.0);
        if (std::abs(idRec - in.gasket_groove_id_mm) < 1.0 &&
            std::abs(odRec - in.gasket_groove_od_mm) < 1.0) {
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found);
}
