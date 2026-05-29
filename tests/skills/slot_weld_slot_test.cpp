// @lat: [[process/test-strategy#compound macro]]
//
// slot_weld_slot — stadium-shaped through slot (box + 2 end semicircles).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/slot_weld_slot.hpp"

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

// ─── 1. Apply produces valid shape with multiple sub-features ────────────
TEST(SkillSlotWeldSlot, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 6.0);
    const int faces0 = stock->faceCount();

    skill::slot_weld_slot::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 40.0;
    in.position_y_mm = 20.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.slot_l        = 30.0;
    in.slot_w        = 8.0;
    in.slot_dir      = gp_Dir(1, 0, 0);

    auto out = skill::slot_weld_slot::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Stadium area = boxLen × slot_w + π × (slot_w/2)²
    const double r = in.slot_w / 2.0;
    const double boxLen = in.slot_l - in.slot_w;          // 22 mm
    const double stadium = boxLen * in.slot_w + M_PI * r * r;
    const double approx = stadium * 6.0;
    const double removed = volumeOf(stock->shape()) -
                           volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.15);

    EXPECT_NE(out.workpiece->faceCount(), faces0);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillSlotWeldSlot, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 6.0);

    // 2a. slot_l < 2 × slot_w
    skill::slot_weld_slot::Input bad1;
    bad1.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad1.position_x_mm = 40.0;
    bad1.position_y_mm = 20.0;
    bad1.slot_l        = 10.0;    // 10 < 2 × 8 = 16
    bad1.slot_w        = 8.0;
    auto r1 = skill::slot_weld_slot::validate(*stock, bad1);
    EXPECT_FALSE(r1.passed);

    // 2b. slot_w < 3 mm
    skill::slot_weld_slot::Input bad2 = bad1;
    bad2.slot_l        = 20.0;
    bad2.slot_w        = 2.0;
    auto r2 = skill::slot_weld_slot::validate(*stock, bad2);
    EXPECT_FALSE(r2.passed);

    EXPECT_THROW(skill::slot_weld_slot::apply(*stock, bad1),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind & subfeature_count == 3 ─────────
TEST(SkillSlotWeldSlot, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 6.0);
    skill::slot_weld_slot::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0;
    in.position_y_mm = 20.0;
    in.slot_l        = 30.0;
    in.slot_w        = 8.0;
    in.slot_dir      = gp_Dir(1, 0, 0);
    auto out = skill::slot_weld_slot::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("slot_weld_slot"));
    EXPECT_TRUE(pat.at("is_compound").get<bool>());
    EXPECT_EQ(pat.at("subfeature_count").get<int>(), 3);
    EXPECT_NEAR(pat.at("slot_l").get<double>(), 30.0, 1e-6);
    EXPECT_NEAR(pat.at("slot_w").get<double>(), 8.0, 1e-6);
    EXPECT_TRUE(pat.contains("corner_r_mm"));
    EXPECT_TRUE(pat.contains("straight_len_mm"));
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillSlotWeldSlot, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 6.0);
    skill::slot_weld_slot::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0;
    in.position_y_mm = 20.0;
    in.slot_l        = 30.0;
    in.slot_w        = 8.0;
    in.slot_dir      = gp_Dir(1, 0, 0);
    auto out = skill::slot_weld_slot::apply(*stock, in);

    auto cands = skill::slot_weld_slot::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params.at("slot_l").get<double>(),
                30.0, 1e-6);
}

// ─── 5. Slot geometry: corner_r = slot_w/2 AND straight_len = slot_l−slot_w
TEST(SkillSlotWeldSlot, SlotGeometryRelations)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 6.0);
    skill::slot_weld_slot::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0;
    in.position_y_mm = 20.0;
    in.slot_l        = 24.0;
    in.slot_w        = 6.0;
    in.slot_dir      = gp_Dir(1, 0, 0);
    auto out = skill::slot_weld_slot::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_NEAR(pat.at("corner_r_mm").get<double>(), in.slot_w / 2.0, 1e-6);
    EXPECT_NEAR(pat.at("straight_len_mm").get<double>(),
                in.slot_l - in.slot_w, 1e-6);
}
