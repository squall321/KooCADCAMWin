// @lat: [[process/test-strategy#compound feature]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/din_rail_mount_slot.hpp"

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

// ─── 1. ApplyProducesValidShapeWithMultipleSubFeatures ──────────────────
TEST(SkillDinRailMountSlot, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    // Stock: 150 long × 80 wide × 12 thick — plenty for DIN35 + 2 holes.
    auto stock = skill::createCuboidStock(150.0, 80.0, 12.0);

    skill::din_rail_mount_slot::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.start_x_mm     = 10.0;
    in.start_y_mm     = 40.0;
    in.rail_dir       = gp_Dir(1.0, 0.0, 0.0);
    in.axis_dir       = gp_Dir(0.0, 0.0, -1.0);
    in.rail_length_mm = 120.0;
    in.plate_t_mm     = 12.0;

    auto out = skill::din_rail_mount_slot::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected stock removal: receiver groove + relief slot + 2 through holes.
    // 120 × 35 × 7.5 + 120 × 27 × 1.5 + 2 × π × 2.25² × 12
    //   = 31500 + 4860 + 381.7 ≈ 36741 mm³
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 30000.0);
    EXPECT_LT(removed, 50000.0);

    // Face count grew — 2 rectangular slot floors + 4 walls each + 2 holes.
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount() + 5);
}

// ─── 2. ValidateRejectsBadInput ─────────────────────────────────────────
TEST(SkillDinRailMountSlot, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(150.0, 80.0, 12.0);

    skill::din_rail_mount_slot::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm     = 10.0;
    in.start_y_mm     = 40.0;
    in.rail_dir       = gp_Dir(1.0, 0.0, 0.0);
    in.rail_length_mm = 40.0;    // < 60 mm → DFM-DIN35
    in.plate_t_mm     = 12.0;

    auto r = skill::din_rail_mount_slot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DIN35") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::din_rail_mount_slot::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. SignatureRecordsCompoundKind ────────────────────────────────────
TEST(SkillDinRailMountSlot, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(150.0, 80.0, 12.0);
    skill::din_rail_mount_slot::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm     = 10.0;
    in.start_y_mm     = 40.0;
    in.rail_dir       = gp_Dir(1.0, 0.0, 0.0);
    in.rail_length_mm = 120.0;
    in.plate_t_mm     = 12.0;
    auto out = skill::din_rail_mount_slot::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("din_rail_mount_slot"));
    EXPECT_TRUE(pat.at("is_compound").get<bool>());
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(pat.at("subfeature_count").get<int>(), 4);
    EXPECT_EQ(pat.at("standard").get<std::string>(),
              std::string("DIN EN 60715 (top-hat 35x7.5)"));
}

// ─── 4. RecognizeMetadataReplay ─────────────────────────────────────────
TEST(SkillDinRailMountSlot, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(150.0, 80.0, 12.0);
    skill::din_rail_mount_slot::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm     = 10.0;
    in.start_y_mm     = 40.0;
    in.rail_dir       = gp_Dir(1.0, 0.0, 0.0);
    in.rail_length_mm = 120.0;
    in.plate_t_mm     = 12.0;
    auto out = skill::din_rail_mount_slot::apply(*stock, in);

    auto cands = skill::din_rail_mount_slot::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params.at("rail_length_mm").get<double>(),
                120.0, 1e-6);
}

// ─── 5. specific: receiver groove wider than finger relief (DIN35 spec) ──
TEST(SkillDinRailMountSlot, ReceiverWiderThanFingerRelief)
{
    auto stock = skill::createCuboidStock(150.0, 80.0, 12.0);
    skill::din_rail_mount_slot::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm     = 10.0;
    in.start_y_mm     = 40.0;
    in.rail_dir       = gp_Dir(1.0, 0.0, 0.0);
    in.rail_length_mm = 120.0;
    in.plate_t_mm     = 12.0;
    auto out = skill::din_rail_mount_slot::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    const double recvW = pat.at("receiver_w_mm").get<double>();
    const double finW  = pat.at("finger_w_mm").get<double>();
    EXPECT_GT(recvW, finW);
    EXPECT_NEAR(recvW, 35.0, 1e-6);   // DIN top-hat rail width
    EXPECT_NEAR(finW,  27.0, 1e-6);

    // Two retention holes must be inside the rail span and spaced enough.
    const double holeA = pat.at("hole_offset_A_mm").get<double>();
    const double holeB = pat.at("hole_offset_B_mm").get<double>();
    EXPECT_GT(holeA, 0.0);
    EXPECT_LT(holeB, in.rail_length_mm);
    EXPECT_GT(holeB - holeA, 25.0);   // spacing ≥ 25 mm
}
