// @lat: [[process/test-strategy#skill round-trip]]
//
// anchor_pad_compound — REAL geometric verification.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/anchor_pad_compound.hpp"

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

skill::anchor_pad_compound::Input goodInput()
{
    skill::anchor_pad_compound::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm           = 130.0;
    in.center_y_mm           = 130.0;
    in.axis_dir              = gp_Dir(0, 0, 1);
    in.plate_length_mm       = 240.0;
    in.plate_width_mm        = 240.0;
    in.plate_thickness_mm    = 24.0;
    in.stud_dia_mm           = 20.0;
    in.stud_height_mm        = 60.0;
    in.stud_pitch_x_mm       = 160.0;
    in.stud_pitch_y_mm       = 160.0;
    in.conduit_dia_mm        = 40.0;
    in.leveling_nut_dia_mm   = 32.0;
    in.leveling_nut_depth_mm = 6.0;
    return in;
}

}  // namespace

// ─── 1. Apply produces REAL volume delta matching expected geometry ───────
TEST(SkillAnchorPadCompound, ApplyProducesExpectedDelta)
{
    // Thin pad base; anchor pad fuses on top of it.
    auto stock = skill::createCuboidStock(260.0, 260.0, 4.0);
    ASSERT_FALSE(stock->shape().IsNull());

    const double volBefore = volumeOf(stock->shape());
    const int facesBefore  = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::anchor_pad_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected ADDED = plate + 4 studs.
    const double vPlate = in.plate_length_mm * in.plate_width_mm *
                          in.plate_thickness_mm;
    const double rStud  = in.stud_dia_mm / 2.0;
    const double vStud  = 4.0 * M_PI * rStud * rStud * in.stud_height_mm;
    // Expected REMOVED = conduit + 4 leveling-nut counterbores.
    const double rCond  = in.conduit_dia_mm / 2.0;
    const double vCond  = M_PI * rCond * rCond * in.plate_thickness_mm;
    const double rLO    = in.leveling_nut_dia_mm / 2.0;
    const double rLI    = in.stud_dia_mm / 2.0 + 0.4;
    const double vLev   = 4.0 * M_PI * (rLO * rLO - rLI * rLI) *
                          in.leveling_nut_depth_mm;
    const double expectedDelta = (vPlate + vStud) - (vCond + vLev);

    const double actualDelta = volumeOf(out.workpiece->shape()) - volBefore;
    EXPECT_GT(actualDelta, 0.0);
    EXPECT_NEAR(actualDelta, expectedDelta, std::abs(expectedDelta) * 0.20)
        << "actual=" << actualDelta << " expected≈" << expectedDelta;

    // Face count grew (≥ 4 stud cyl + conduit + leveling nuts).
    EXPECT_GT(out.workpiece->faceCount(), facesBefore + 4);
}

// ─── 2. Validate rejects bad spacing + apply throws ───────────────────────
TEST(SkillAnchorPadCompound, ValidateRejectsTooSmallSpacing)
{
    auto stock = skill::createCuboidStock(260.0, 260.0, 4.0);
    auto in    = goodInput();
    in.stud_pitch_x_mm = 40.0;   // < 3 × stud_dia (= 60)

    auto r = skill::anchor_pad_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-ACI-318"));
    EXPECT_THROW(skill::anchor_pad_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature is_compound=true & params round-trip ────────────────────
TEST(SkillAnchorPadCompound, SignatureCompoundAndParamsRoundTrip)
{
    auto stock = skill::createCuboidStock(260.0, 260.0, 4.0);
    auto in    = goodInput();
    auto out   = skill::anchor_pad_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("anchor_pad_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 10);
    EXPECT_EQ(out.signature.pattern.value("stud_count", 0), 4);

    EXPECT_NEAR(out.signature.params.value("stud_dia_mm", 0.0),
                in.stud_dia_mm, 1e-6);
}

// ─── 4. Recognize returns a candidate with confidence > 0.5 ───────────────
TEST(SkillAnchorPadCompound, RecognizeFindsCompound)
{
    auto stock = skill::createCuboidStock(260.0, 260.0, 4.0);
    auto in    = goodInput();
    auto out   = skill::anchor_pad_compound::apply(*stock, in);

    auto cands = skill::anchor_pad_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    bool ok = false;
    for (const auto& c : cands) {
        if (c.confidence > 0.5) { ok = true; break; }
    }
    EXPECT_TRUE(ok);
}

// ─── 5. Feature-specific: stud diameter recovered correctly ───────────────
TEST(SkillAnchorPadCompound, RecoveredStudDiameterMatches)
{
    auto stock = skill::createCuboidStock(260.0, 260.0, 4.0);
    auto in    = goodInput();
    in.stud_dia_mm = 16.0;
    in.leveling_nut_dia_mm = 28.0;
    in.stud_pitch_x_mm     = 100.0;     // ≥ 3 × 16 = 48
    in.stud_pitch_y_mm     = 100.0;
    auto out   = skill::anchor_pad_compound::apply(*stock, in);

    auto cands = skill::anchor_pad_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found = false;
    for (const auto& c : cands) {
        const double dia = c.recovered_params.value("stud_dia_mm", 0.0);
        if (std::abs(dia - in.stud_dia_mm) < 0.6) { found = true; break; }
    }
    EXPECT_TRUE(found);
}
