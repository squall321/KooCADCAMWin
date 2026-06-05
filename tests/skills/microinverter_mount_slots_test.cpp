// @lat: [[process/test-strategy#microinverter_mount_slots]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/microinverter_mount_slots.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::microinverter_mount_slots::Input goodInput()
{
    skill::microinverter_mount_slots::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy       = gp_Pnt(45.0, 45.0, 0.0);
    in.slot_length_mm  = 20.0;
    in.slot_width_mm   = 7.0;
    in.slot_spacing_mm = 30.0;
    in.notch_width_mm  = 8.0;
    in.notch_depth_mm  = 6.0;
    return in;
}
}  // namespace

TEST(SkillMicroinverterMountSlots, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(90.0, 90.0, 12.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::microinverter_mount_slots::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillMicroinverterMountSlots, ValidateRejectsDegenerateSlot)
{
    auto stock = skill::createCuboidStock(90.0, 90.0, 12.0);
    auto in = goodInput();
    in.slot_length_mm = 7.0;   // == slot_width_mm

    auto r = skill::microinverter_mount_slots::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SLOTGEOM") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::microinverter_mount_slots::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillMicroinverterMountSlots, ValidateRejectsOverlapSpacing)
{
    auto stock = skill::createCuboidStock(90.0, 90.0, 12.0);
    auto in = goodInput();
    in.slot_spacing_mm = 5.0;   // <= slot_width_mm

    auto r = skill::microinverter_mount_slots::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPACING") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillMicroinverterMountSlots, SignatureCompound)
{
    auto stock = skill::createCuboidStock(90.0, 90.0, 12.0);
    auto in    = goodInput();
    auto out   = skill::microinverter_mount_slots::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("solar_feature_type", std::string()),
              std::string("microinverter_mount_slots"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 7);
}

TEST(SkillMicroinverterMountSlots, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(90.0, 90.0, 12.0);
    auto in    = goodInput();
    auto out   = skill::microinverter_mount_slots::apply(*stock, in);
    auto cands = skill::microinverter_mount_slots::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
