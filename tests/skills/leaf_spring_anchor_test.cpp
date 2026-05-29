// @lat: [[process/test-strategy#compound macro]]
//
// leaf_spring_anchor — compound: slot + retention undercut + chamfer entry.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/leaf_spring_anchor.hpp"

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

// ─── 1. Apply removes slot + undercut + chamfer volumes ───────────────────
TEST(SkillLeafSpringAnchor, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 12.0);

    skill::leaf_spring_anchor::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 30.0;
    in.position_y_mm = 20.0;
    in.slot_length_mm = 20.0;
    in.slot_width_mm  = 6.0;
    in.depth_mm       = 6.0;

    const int origFaces = stock->faceCount();
    auto out = skill::leaf_spring_anchor::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected:
    //  slot  = 20 × 6 × 6 = 720
    //  undercut extra = 20 × (2×1) × 1 = 40
    //  chamfer extra ≈ (20+6) × 2 × 0.5 × 0.5 = 13
    const double slotV = 20.0 * 6.0 * 6.0;
    const double underV = 20.0 * 2.0 * 1.0;
    const double chamferV = (20.0 + 6.0) * 2.0 * 0.5 * 0.5;
    const double expected = slotV + underV + chamferV;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.20);

    EXPECT_GT(out.workpiece->faceCount(), origFaces);
}

// ─── 2. Validate rejects sub-min slot width ───────────────────────────────
TEST(SkillLeafSpringAnchor, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 12.0);

    skill::leaf_spring_anchor::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 20.0;
    in.slot_length_mm = 20.0;
    in.slot_width_mm  = 2.0;   // < 4 mm → DFM-ANCHOR-LIP
    in.depth_mm       = 6.0;

    auto r = skill::leaf_spring_anchor::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ANCHOR-LIP") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::leaf_spring_anchor::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records compound metadata ────────────────────────────────
TEST(SkillLeafSpringAnchor, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 12.0);

    skill::leaf_spring_anchor::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 20.0;
    in.slot_length_mm = 20.0;
    in.slot_width_mm  = 6.0;
    in.depth_mm       = 6.0;
    auto out = skill::leaf_spring_anchor::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("leaf_spring_anchor"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_NEAR(out.signature.pattern.at("lip_overhang_mm").get<double>(),
                1.0, 1e-6);
}

// ─── 4. Recognize replays with confidence 1.0 ──────────────────────────────
TEST(SkillLeafSpringAnchor, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 12.0);

    skill::leaf_spring_anchor::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 20.0;
    in.slot_length_mm = 20.0;
    in.slot_width_mm  = 6.0;
    in.depth_mm       = 6.0;
    auto out = skill::leaf_spring_anchor::apply(*stock, in);

    auto cands = skill::leaf_spring_anchor::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── 5. Specific: undercut width = slot_width + 2 × overhang ──────────────
TEST(SkillLeafSpringAnchor, UndercutWidthMatchesOverhangFormula)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 12.0);

    skill::leaf_spring_anchor::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 20.0;
    in.slot_length_mm = 20.0;
    in.slot_width_mm  = 6.0;
    in.depth_mm       = 6.0;
    auto out = skill::leaf_spring_anchor::apply(*stock, in);

    const double slotW   = out.signature.pattern.at("slot_width_mm").get<double>();
    const double underW  = out.signature.pattern.at("undercut_width_mm").get<double>();
    const double overhang = out.signature.pattern.at("lip_overhang_mm").get<double>();
    EXPECT_NEAR(underW, slotW + 2.0 * overhang, 1e-6);
    EXPECT_GT(underW, slotW);     // undercut MUST be wider (this is the retention geometry)
}
