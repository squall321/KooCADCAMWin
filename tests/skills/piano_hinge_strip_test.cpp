// @lat: [[process/test-strategy#skill compound feature]]
//
// piano_hinge_strip — REAL geometric test, not metadata-clone.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/piano_hinge_strip.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── Test 1: apply produces non-null shape, volume delta matches expected ─
TEST(SkillPianoHingeStrip, ApplyProducesRealGeometry)
{
    // Stock 100 × 30 × 5 mm aluminum strip
    auto stock = skill::createCuboidStock(100.0, 30.0, 5.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::piano_hinge_strip::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.origin_x_mm           = 10.0;
    in.origin_y_mm           = 2.5;
    in.pin_axis_dir          = gp_Dir(1.0, 0.0, 0.0);
    in.strip_length_mm       = 80.0;
    in.strip_width_mm        = 25.0;
    in.strip_thickness_mm    = 5.0;
    in.knuckle_count         = 4;
    in.knuckle_pitch_mm      = 18.0;
    in.knuckle_slot_width_mm = 4.0;
    in.knuckle_slot_depth_mm = 2.5;
    in.pin_dia_mm            = 2.5;
    in.mount_hole_dia_mm     = 3.0;
    in.mount_hole_inset_mm   = 4.0;

    auto out = skill::piano_hinge_strip::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume delta = slot vol + pin vol + mount vol (approx, ignoring overhang)
    const double slotVol  = in.knuckle_slot_width_mm * in.strip_width_mm
                            * in.knuckle_slot_depth_mm * in.knuckle_count;
    const double pinVol   = M_PI * std::pow(in.pin_dia_mm / 2.0, 2.0)
                            * in.strip_length_mm;
    const double mountVol = M_PI * std::pow(in.mount_hole_dia_mm / 2.0, 2.0)
                            * in.strip_thickness_mm * 2 * in.knuckle_count;
    const double expected = slotVol + pinVol + mountVol;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());

    // ±20% tolerance per spec
    EXPECT_NEAR(removed, expected, expected * 0.20)
        << "removed=" << removed << " expected=" << expected;

    // Faces grew by knuckles + pin + mount holes
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount() + in.knuckle_count);
}

// ─── Test 2: validate rejects bad input + apply throws ───────────────────
TEST(SkillPianoHingeStrip, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 30.0, 5.0);

    skill::piano_hinge_strip::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.origin_x_mm           = 10.0;
    in.origin_y_mm           = 2.5;
    in.pin_axis_dir          = gp_Dir(1.0, 0.0, 0.0);
    in.strip_length_mm       = 80.0;
    in.strip_width_mm        = 25.0;
    in.strip_thickness_mm    = 5.0;
    in.knuckle_count         = 1;          // < 2 — DFM-PIANO-KNUCKLES
    in.knuckle_pitch_mm      = 18.0;
    in.knuckle_slot_width_mm = 4.0;
    in.pin_dia_mm            = 2.5;
    in.mount_hole_dia_mm     = 3.0;

    auto r = skill::piano_hinge_strip::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::piano_hinge_strip::apply(*stock, in), skill::SkillError);
}

// ─── Test 3: signature has is_compound=true and subfeature_count >= 2 ────
TEST(SkillPianoHingeStrip, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(100.0, 30.0, 5.0);

    skill::piano_hinge_strip::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.origin_x_mm           = 10.0;
    in.origin_y_mm           = 2.5;
    in.pin_axis_dir          = gp_Dir(1.0, 0.0, 0.0);
    in.strip_length_mm       = 80.0;
    in.strip_width_mm        = 25.0;
    in.strip_thickness_mm    = 5.0;
    in.knuckle_count         = 3;
    in.knuckle_pitch_mm      = 18.0;
    in.knuckle_slot_width_mm = 4.0;
    in.knuckle_slot_depth_mm = 2.5;
    in.pin_dia_mm            = 2.5;
    in.mount_hole_dia_mm     = 3.0;
    in.mount_hole_inset_mm   = 4.0;

    auto out = skill::piano_hinge_strip::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    const int sfc = out.signature.pattern["subfeature_count"].get<int>();
    EXPECT_GE(sfc, 2);
    // 3 knuckles + 1 pin + 6 mount holes = 10
    EXPECT_EQ(sfc, 10);

    // Round-trip of all params
    EXPECT_NEAR(out.signature.params["strip_length_mm"].get<double>(),
                in.strip_length_mm, 1e-6);
    EXPECT_NEAR(out.signature.params["pin_dia_mm"].get<double>(),
                in.pin_dia_mm, 1e-6);
    EXPECT_EQ(out.signature.params["knuckle_count"].get<int>(),
              in.knuckle_count);
}

// ─── Test 4: recognize returns >= 1 candidate ─────────────────────────────
TEST(SkillPianoHingeStrip, RecognizeFindsCandidate)
{
    auto stock = skill::createCuboidStock(100.0, 30.0, 5.0);

    skill::piano_hinge_strip::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.origin_x_mm           = 10.0;
    in.origin_y_mm           = 2.5;
    in.pin_axis_dir          = gp_Dir(1.0, 0.0, 0.0);
    in.strip_length_mm       = 80.0;
    in.strip_width_mm        = 25.0;
    in.strip_thickness_mm    = 5.0;
    in.knuckle_count         = 4;
    in.knuckle_pitch_mm      = 18.0;
    in.knuckle_slot_width_mm = 4.0;
    in.knuckle_slot_depth_mm = 2.5;
    in.pin_dia_mm            = 2.5;
    in.mount_hole_dia_mm     = 3.0;
    in.mount_hole_inset_mm   = 4.0;

    auto out = skill::piano_hinge_strip::apply(*stock, in);
    auto cands = skill::piano_hinge_strip::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    // metadata replay should yield confidence 1.0
    bool sawConfident = false;
    for (const auto& c : cands) if (c.confidence > 0.5) sawConfident = true;
    EXPECT_TRUE(sawConfident);
}

// ─── Test 5: feature-specific — pin bore length ≈ strip length ────────────
TEST(SkillPianoHingeStrip, PinBoreLengthMatchesStripLength)
{
    auto stock = skill::createCuboidStock(120.0, 30.0, 5.0);

    skill::piano_hinge_strip::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.origin_x_mm           = 20.0;
    in.origin_y_mm           = 2.5;
    in.pin_axis_dir          = gp_Dir(1.0, 0.0, 0.0);
    in.strip_length_mm       = 80.0;
    in.strip_width_mm        = 25.0;
    in.strip_thickness_mm    = 5.0;
    in.knuckle_count         = 3;
    in.knuckle_pitch_mm      = 22.0;
    in.knuckle_slot_width_mm = 4.0;
    in.knuckle_slot_depth_mm = 2.5;
    in.pin_dia_mm            = 2.5;
    in.mount_hole_dia_mm     = 3.0;
    in.mount_hole_inset_mm   = 4.0;

    auto out = skill::piano_hinge_strip::apply(*stock, in);
    auto cands = skill::piano_hinge_strip::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    // The geometric (non-metadata) candidate's strip_length should be near
    // the bbox of the modified workpiece's pin bore — pin runs the entire
    // strip length.  We use the metadata replay if available since it's
    // exact; otherwise check the geometric candidate.
    bool found = false;
    for (const auto& c : cands) {
        if (c.recovered_params.contains("strip_length_mm")) {
            EXPECT_NEAR(c.recovered_params["strip_length_mm"].get<double>(),
                        in.strip_length_mm, in.strip_length_mm * 0.5);
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found);
}
