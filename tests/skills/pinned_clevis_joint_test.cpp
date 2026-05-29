// @lat: [[process/test-strategy#skill round-trip]]
//
// pinned_clevis_joint skill — REAL geometric verification.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pinned_clevis_joint.hpp"

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

skill::pinned_clevis_joint::Input goodInput()
{
    skill::pinned_clevis_joint::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm           = 30.0;       // stock spans 0..60 along X
    in.center_y_mm           = 20.0;       // stock spans 0..40 along Y
    in.axis_dir              = gp_Dir(0, 0, -1);
    in.block_length_mm       = 60.0;
    in.block_width_mm        = 40.0;
    in.block_height_mm       = 40.0;
    in.tongue_slot_width_mm  = 14.0;
    in.tongue_slot_depth_mm  = 26.0;
    in.pin_hole_dia_mm       = 12.0;
    in.pin_hole_z_offset_mm  = 14.0;
    in.cotter_pin_dia_mm     = 3.0;
    in.cotter_pin_offset_mm  = 10.0;
    return in;
}

}  // namespace

// ─── 1. Apply produces REAL volume delta matching expected geometry ───────
TEST(SkillPinnedClevisJoint, ApplyRemovesExpectedVolume)
{
    // Stock block 60 × 40 × 40 (matches block dims).
    auto stock = skill::createCuboidStock(60.0, 40.0, 40.0);
    ASSERT_FALSE(stock->shape().IsNull());

    const double volBefore = volumeOf(stock->shape());
    const int facesBefore  = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::pinned_clevis_joint::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected REMOVED ≈ slot + pin + 2 cotter (upper bound — ignore overlaps).
    const double vSlot   = in.block_length_mm * in.tongue_slot_width_mm *
                           in.tongue_slot_depth_mm;
    const double rPin    = in.pin_hole_dia_mm    / 2.0;
    const double rCotter = in.cotter_pin_dia_mm  / 2.0;
    const double vPin    = M_PI * rPin * rPin * in.block_length_mm;
    const double vCotter = 2.0 * M_PI * rCotter * rCotter * in.block_width_mm;
    const double expectedRemoved = vSlot + vPin + vCotter;

    const double actualRemoved = volBefore - volumeOf(out.workpiece->shape());
    EXPECT_GT(actualRemoved, 0.0);
    // Tolerance ±25 % to absorb pin/slot intersection overlap.
    EXPECT_NEAR(actualRemoved, expectedRemoved, expectedRemoved * 0.25)
        << "removed=" << actualRemoved << " expected≈" << expectedRemoved;

    // Face count grew (slot adds 5 faces, pin adds 1 cyl, cotters add 2).
    EXPECT_GT(out.workpiece->faceCount(), facesBefore + 2);
}

// ─── 2. Validate rejects bad input + apply throws ─────────────────────────
TEST(SkillPinnedClevisJoint, ValidateRejectsThinEar)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 40.0);
    auto in = goodInput();
    // Force a thin ear: very wide slot in narrow block.
    in.tongue_slot_width_mm = 36.0;  // ear thickness = (40 - 36)/2 = 2.0 mm
                                     // vs min 0.6 × 12 = 7.2 mm
    auto r = skill::pinned_clevis_joint::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-DIN-71752"));
    EXPECT_THROW(skill::pinned_clevis_joint::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature is_compound=true, subfeature_count >= 2 ─────────────────
TEST(SkillPinnedClevisJoint, SignatureCompoundAndParamsRoundTrip)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 40.0);
    auto in = goodInput();
    auto out = skill::pinned_clevis_joint::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("pinned_clevis_joint"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_GE(out.signature.pattern.value("subfeature_count", 0), 2);

    EXPECT_NEAR(out.signature.params.value("pin_hole_dia_mm", 0.0),
                in.pin_hole_dia_mm, 1e-6);
    EXPECT_NEAR(out.signature.params.value("tongue_slot_width_mm", 0.0),
                in.tongue_slot_width_mm, 1e-6);
}

// ─── 4. Recognize returns a candidate with confidence > 0.5 ───────────────
TEST(SkillPinnedClevisJoint, RecognizeFindsCompound)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 40.0);
    auto in    = goodInput();
    auto out   = skill::pinned_clevis_joint::apply(*stock, in);

    auto cands = skill::pinned_clevis_joint::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    bool ok = false;
    for (const auto& c : cands) {
        if (c.confidence > 0.5) { ok = true; break; }
    }
    EXPECT_TRUE(ok);
}

// ─── 5. Feature-specific: recovered pin diameter matches ──────────────────
TEST(SkillPinnedClevisJoint, RecoveredPinDiameterMatches)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 40.0);
    auto in    = goodInput();
    in.pin_hole_dia_mm = 10.0;
    auto out   = skill::pinned_clevis_joint::apply(*stock, in);

    auto cands = skill::pinned_clevis_joint::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found = false;
    for (const auto& c : cands) {
        const double dia = c.recovered_params.value("pin_hole_dia_mm", 0.0);
        if (std::abs(dia - in.pin_hole_dia_mm) < 0.5) { found = true; break; }
    }
    EXPECT_TRUE(found);
}
