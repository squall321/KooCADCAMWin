// @lat: [[process/test-strategy#compound macro]]
//
// spiral_back_up_ring_groove — compound: O-ring groove + adjacent back-up slot.
//
// Tests (5):
//   1. ApplyProducesValidShapeWithMultipleSubFeatures.
//   2. ValidateRejectsBadInput — unknown AS568 + invalid position string.
//   3. SignatureRecordsCompoundKind.
//   4. RecognizeMetadataReplay.
//   5. BackUpSlotOffsetMatchesPositionField — downstream → +Z offset.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/spiral_back_up_ring_groove.hpp"

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

// ─── 1. Apply produces valid shape with multiple sub-features ─────────────
TEST(SkillSpiralBackupRingGroove, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCylindricalStock(40.0, 40.0);

    skill::spiral_back_up_ring_groove::Input in;
    in.bore_or_shaft_dia_mm = 40.0;
    in.position_z_mm        = 20.0;
    in.ring_size            = "-210";   // CS 3.53
    in.position             = "downstream";

    const int facesBefore = stock->faceCount();
    auto out = skill::spiral_back_up_ring_groove::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const int facesAfter = out.workpiece->faceCount();
    EXPECT_GT(facesAfter, facesBefore);

    const double cs       = 3.53;
    const double depth    = 0.74 * cs;
    const double widthO   = 1.45 * cs;
    const double widthBu  = 0.5  * widthO;
    const double shaftR   = 20.0;
    const double grooveR  = shaftR - depth;
    const double v_ring   = M_PI * (shaftR * shaftR - grooveR * grooveR);
    const double rectVol  = v_ring * (widthO + widthBu);
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.9 * rectVol);
    EXPECT_LT(removed, 1.3 * rectVol);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillSpiralBackupRingGroove, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(40.0, 40.0);

    skill::spiral_back_up_ring_groove::Input bad;
    bad.bore_or_shaft_dia_mm = 40.0;
    bad.position_z_mm        = 20.0;
    bad.ring_size            = "-XYZ";
    bad.position             = "downstream";
    auto r1 = skill::spiral_back_up_ring_groove::validate(*stock, bad);
    EXPECT_FALSE(r1.passed);

    skill::spiral_back_up_ring_groove::Input wrongDir;
    wrongDir.bore_or_shaft_dia_mm = 40.0;
    wrongDir.position_z_mm        = 20.0;
    wrongDir.ring_size            = "-210";
    wrongDir.position             = "sideways";   // invalid
    auto r2 = skill::spiral_back_up_ring_groove::validate(*stock, wrongDir);
    EXPECT_FALSE(r2.passed);

    EXPECT_THROW(skill::spiral_back_up_ring_groove::apply(*stock, bad),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind ────────────────────────────────────
TEST(SkillSpiralBackupRingGroove, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCylindricalStock(40.0, 40.0);

    skill::spiral_back_up_ring_groove::Input in;
    in.bore_or_shaft_dia_mm = 40.0;
    in.position_z_mm        = 20.0;
    in.ring_size            = "-110";
    in.position             = "downstream";

    auto out = skill::spiral_back_up_ring_groove::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("spiral_back_up_ring_groove"));
    EXPECT_EQ(pat.at("is_compound").get<bool>(), true);
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
}

// ─── 4. Recognize metadata replay ──────────────────────────────────────────
TEST(SkillSpiralBackupRingGroove, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 40.0);

    skill::spiral_back_up_ring_groove::Input in;
    in.bore_or_shaft_dia_mm = 40.0;
    in.position_z_mm        = 20.0;
    in.ring_size            = "-210";
    in.position             = "downstream";

    auto out   = skill::spiral_back_up_ring_groove::apply(*stock, in);
    auto cands = skill::spiral_back_up_ring_groove::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("spiral_back_up_ring_groove"));
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("position").get<std::string>(),
              std::string("downstream"));
}

// ─── 5. Back-up slot is offset from O-ring center per `position` ──────────
TEST(SkillSpiralBackupRingGroove, BackUpSlotOffsetMatchesPositionField)
{
    auto stock = skill::createCylindricalStock(40.0, 40.0);

    skill::spiral_back_up_ring_groove::Input downstream;
    downstream.bore_or_shaft_dia_mm = 40.0;
    downstream.position_z_mm        = 20.0;
    downstream.ring_size            = "-210";
    downstream.position             = "downstream";
    auto outDown = skill::spiral_back_up_ring_groove::apply(*stock, downstream);

    skill::spiral_back_up_ring_groove::Input upstream = downstream;
    upstream.position = "upstream";
    auto outUp = skill::spiral_back_up_ring_groove::apply(*stock, upstream);

    const double bzDown =
        outDown.signature.pattern.at("backup_center_z_mm").get<double>();
    const double bzUp =
        outUp.signature.pattern.at("backup_center_z_mm").get<double>();

    EXPECT_GT(bzDown, 20.0);   // downstream → above O-ring center
    EXPECT_LT(bzUp,   20.0);   // upstream   → below

    // Symmetry: same magnitude of offset.
    EXPECT_NEAR(std::abs(bzDown - 20.0), std::abs(bzUp - 20.0), 1e-6);
}
