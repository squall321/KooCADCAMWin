// @lat: [[process/test-strategy#cable_carrier_anchor_bracket]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cable_carrier_anchor_bracket.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::cable_carrier_anchor_bracket::Input goodInput()
{
    skill::cable_carrier_anchor_bracket::Input in;
    in.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy            = gp_Pnt(45.0, 35.0, 0.0);   // in 90×90 stock
    in.chain_width_mm       = 30.0;
    in.bolt_slot_len_mm     = 14.0;
    in.bolt_slot_wid_mm     = 6.5;
    in.bolt_slot_spacing_mm = 40.0;
    in.pin_bore_dia_mm      = 8.0;
    return in;
}
}  // namespace

TEST(SkillCableCarrierAnchorBracket, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(90.0, 90.0, 12.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::cable_carrier_anchor_bracket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // 2 slots + pin bore removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillCableCarrierAnchorBracket, ValidateRejectsBadDimensions)
{
    auto stock = skill::createCuboidStock(90.0, 90.0, 12.0);
    auto in = goodInput();
    in.pin_bore_dia_mm = 0.0;

    auto r = skill::cable_carrier_anchor_bracket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::cable_carrier_anchor_bracket::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillCableCarrierAnchorBracket, ValidateRejectsRoundSlot)
{
    auto stock = skill::createCuboidStock(90.0, 90.0, 12.0);
    auto in = goodInput();
    in.bolt_slot_len_mm = 6.0;   // <= bolt_slot_wid_mm (6.5)

    auto r = skill::cable_carrier_anchor_bracket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ROBOTICS-SLOT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillCableCarrierAnchorBracket, SignatureCompoundAnchor)
{
    auto stock = skill::createCuboidStock(90.0, 90.0, 12.0);
    auto in    = goodInput();
    auto out   = skill::cable_carrier_anchor_bracket::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("cable_carrier_anchor_bracket"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("robotics_feature_type", std::string()),
              std::string("energy_chain_end_anchor"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0),
              2 + 1);
}

TEST(SkillCableCarrierAnchorBracket, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(90.0, 90.0, 12.0);
    auto in    = goodInput();
    auto out   = skill::cable_carrier_anchor_bracket::apply(*stock, in);
    auto cands = skill::cable_carrier_anchor_bracket::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
