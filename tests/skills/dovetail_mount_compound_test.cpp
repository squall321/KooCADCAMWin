// @lat: [[process/test-strategy#compound feature]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/dovetail_mount_compound.hpp"

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
TEST(SkillDovetailMountCompound, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    // Stock: long enough for slot + 2 side set-screws (need stock height ≥
    // slot_depth, width ≥ slot_bottom_width + 2 × side margin).
    auto stock = skill::createCuboidStock(120.0, 60.0, 25.0);

    skill::dovetail_mount_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.start_x_mm         = 10.0;
    in.start_y_mm         = 30.0;
    in.slot_dir           = gp_Dir(1.0, 0.0, 0.0);
    in.axis_dir           = gp_Dir(0.0, 0.0, -1.0);
    in.slot_length_mm     = 100.0;
    in.slot_width_mm      = 10.0;
    in.slot_depth_mm      = 8.0;
    in.dovetail_angle_deg = 30.0;

    auto out = skill::dovetail_mount_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume removed > 0.  Trapezoid dovetail volume alone:
    // bot_w = 10 + 2*8*tan30 = 10 + 9.24 = 19.24
    // mean width = (10+19.24)/2 = 14.62
    // dove volume ≈ 14.62 × 8 × 100 = 11696 mm³
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 10000.0);

    // Face count grew significantly (4 angled walls + 2 end caps + bottom +
    // 2 set-screw side cylinders + stop walls).
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount() + 5);
}

// ─── 2. ValidateRejectsBadInput ─────────────────────────────────────────
TEST(SkillDovetailMountCompound, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(120.0, 60.0, 25.0);

    skill::dovetail_mount_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm         = 10.0;
    in.start_y_mm         = 30.0;
    in.slot_dir           = gp_Dir(1.0, 0.0, 0.0);
    in.slot_length_mm     = 100.0;
    in.slot_width_mm      = 10.0;
    in.slot_depth_mm      = 8.0;
    in.dovetail_angle_deg = 60.0;  // > 45° → DFM-DOVETAIL

    auto r = skill::dovetail_mount_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DOVETAIL") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::dovetail_mount_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. SignatureRecordsCompoundKind ────────────────────────────────────
TEST(SkillDovetailMountCompound, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(120.0, 60.0, 25.0);
    skill::dovetail_mount_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm         = 10.0;
    in.start_y_mm         = 30.0;
    in.slot_dir           = gp_Dir(1.0, 0.0, 0.0);
    in.slot_length_mm     = 100.0;
    in.slot_width_mm      = 10.0;
    in.slot_depth_mm      = 8.0;
    in.dovetail_angle_deg = 30.0;
    auto out = skill::dovetail_mount_compound::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("dovetail_mount_compound"));
    EXPECT_TRUE(pat.at("is_compound").get<bool>());
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(pat.at("subfeature_count").get<int>(), 4);
    EXPECT_EQ(pat.at("set_screw_count").get<int>(), 2);
}

// ─── 4. RecognizeMetadataReplay ─────────────────────────────────────────
TEST(SkillDovetailMountCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(120.0, 60.0, 25.0);
    skill::dovetail_mount_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm         = 10.0;
    in.start_y_mm         = 30.0;
    in.slot_dir           = gp_Dir(1.0, 0.0, 0.0);
    in.slot_length_mm     = 100.0;
    in.slot_width_mm      = 12.0;
    in.slot_depth_mm      = 8.0;
    in.dovetail_angle_deg = 30.0;
    auto out = skill::dovetail_mount_compound::apply(*stock, in);

    auto cands = skill::dovetail_mount_compound::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params.at("slot_width_mm").get<double>(),
                12.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params.at("dovetail_angle_deg").get<double>(),
                30.0, 1e-6);
}

// ─── 5. specific: dovetail bottom width > top width per slot geom ────────
TEST(SkillDovetailMountCompound, BottomWidthExceedsTopWidth)
{
    auto stock = skill::createCuboidStock(120.0, 60.0, 25.0);
    skill::dovetail_mount_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm         = 10.0;
    in.start_y_mm         = 30.0;
    in.slot_dir           = gp_Dir(1.0, 0.0, 0.0);
    in.slot_length_mm     = 100.0;
    in.slot_width_mm      = 10.0;
    in.slot_depth_mm      = 8.0;
    in.dovetail_angle_deg = 30.0;
    auto out = skill::dovetail_mount_compound::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    const double topW = pat.at("slot_top_width_mm").get<double>();
    const double botW = pat.at("slot_bot_width_mm").get<double>();
    EXPECT_GT(botW, topW);
    // bot = top + 2 × depth × tan(angle) = 10 + 2 × 8 × tan(30°) = 10 + 9.238
    EXPECT_NEAR(botW - topW,
                2.0 * 8.0 * std::tan(30.0 * M_PI / 180.0), 1e-4);
}
