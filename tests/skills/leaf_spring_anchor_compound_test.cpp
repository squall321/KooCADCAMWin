// @lat: [[process/test-strategy#compound feature]]
//
// leaf_spring_anchor_compound — 4 sub-feature chain:
//   slot + undercut + 2 screw pilots.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/leaf_spring_anchor_compound.hpp"

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

// ─── 1. apply produces a valid shape with multiple sub-features ───────────
TEST(SkillLeafSpringAnchor, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 30.0);

    skill::leaf_spring_anchor_compound::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 40.0;
    in.position_y_mm   = 20.0;
    in.axis_dir        = gp_Dir(0, 0, -1);
    in.slot_length_mm  = 40.0;
    in.slot_width_mm   = 14.0;
    in.slot_depth_mm   = 10.0;
    in.screw_M         = "M4";

    const double v0 = volumeOf(stock->shape());
    const int    n0 = stock->faceCount();
    auto out = skill::leaf_spring_anchor_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Volume removed > slot volume (slot + undercut + 2 pilots).
    const double slotVol = 40.0 * 14.0 * 10.0;
    EXPECT_GT(v0 - v1, slotVol);

    // Face count must grow (4 sub-features).
    EXPECT_GT(out.workpiece->faceCount(), n0);
}

// ─── 2. validate rejects bad input (depth too shallow) ────────────────────
TEST(SkillLeafSpringAnchor, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 30.0);

    skill::leaf_spring_anchor_compound::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 40.0;
    in.position_y_mm   = 20.0;
    in.slot_length_mm  = 40.0;
    in.slot_width_mm   = 16.0;
    in.slot_depth_mm   = 4.0;             // < width / 2 = 8 mm
    in.screw_M         = "M4";

    auto r = skill::leaf_spring_anchor_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundDepth = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LEAF-DEPTH") { foundDepth = true; break; }
    EXPECT_TRUE(foundDepth);

    EXPECT_THROW(skill::leaf_spring_anchor_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. signature records is_compound + subfeature_count >= 2 ────────────
TEST(SkillLeafSpringAnchor, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 30.0);

    skill::leaf_spring_anchor_compound::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 40.0;
    in.position_y_mm   = 20.0;
    in.slot_length_mm  = 36.0;
    in.slot_width_mm   = 12.0;
    in.slot_depth_mm   = 10.0;
    in.screw_M         = "M5";

    auto out = skill::leaf_spring_anchor_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("leaf_spring_anchor_compound"));
    EXPECT_EQ(out.signature.pattern.at("kind"),
              std::string("leaf_spring_anchor_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
    EXPECT_EQ(out.signature.params.at("screw_M"), std::string("M5"));
    EXPECT_NEAR(out.signature.pattern.at("pilot_dia_mm").get<double>(),
                5.5, 1e-9);     // M5 ISO 273 medium clearance
}

// ─── 4. recognize() replays metadata at confidence 1.0 ───────────────────
TEST(SkillLeafSpringAnchor, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 30.0);

    skill::leaf_spring_anchor_compound::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 40.0;
    in.position_y_mm   = 20.0;
    in.slot_length_mm  = 30.0;
    in.slot_width_mm   = 12.0;
    in.slot_depth_mm   = 8.0;
    in.screw_M         = "M3";

    auto out = skill::leaf_spring_anchor_compound::apply(*stock, in);
    auto cands = skill::leaf_spring_anchor_compound::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id,
              std::string("leaf_spring_anchor_compound"));
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params.at("screw_M"), std::string("M3"));
    EXPECT_NEAR(cands[0].recovered_params.at("slot_length_mm").get<double>(),
                30.0, 1e-9);
}

// ─── 5. specific: 2 screw pilots, spacing = slot_length / 2 ───────────────
TEST(SkillLeafSpringAnchor, TwoScrewPilotsSpacedHalfSlotLength)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 30.0);

    skill::leaf_spring_anchor_compound::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 40.0;
    in.position_y_mm   = 20.0;
    in.slot_length_mm  = 40.0;
    in.slot_width_mm   = 14.0;
    in.slot_depth_mm   = 10.0;
    in.screw_M         = "M4";

    auto out = skill::leaf_spring_anchor_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern.at("screw_count").get<int>(), 2);
    // Offset = slot_length / 4 → total spacing = slot_length / 2
    const double offset = out.signature.pattern.at("screw_offset_mm").get<double>();
    EXPECT_NEAR(offset, 40.0 / 4.0, 1e-9);
    EXPECT_NEAR(2.0 * offset, 40.0 / 2.0, 1e-9);
    // M4 clearance = 4.5 mm (ISO 273)
    EXPECT_NEAR(out.signature.pattern.at("pilot_dia_mm").get<double>(),
                4.5, 1e-9);
    EXPECT_NEAR(skill::leaf_spring_anchor_compound::pilotForScrew("M4"),
                4.5, 1e-9);
    EXPECT_EQ(skill::leaf_spring_anchor_compound::pilotForScrew("M99"), 0.0);
}
