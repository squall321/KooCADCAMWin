// @lat: [[process/test-strategy#compound macro]]
//
// ball_screw_nut_pocket — COMPOUND macro: nut OD pocket + 4 mounting holes +
// shaft thru-pocket.  Tests:
//   1. apply produces valid shape with multiple sub-features.
//   2. validate rejects bad input.
//   3. signature records compound kind, subfeature_count >= 2.
//   4. recognize metadata replay (confidence 1.0).
//   5. slip_dia_mm == nut_od + 2 × slip_clearance (the spec invariant).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ball_screw_nut_pocket.hpp"

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
TEST(SkillBallScrewNutPocket, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 50.0);

    skill::ball_screw_nut_pocket::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm       = 40.0;
    in.position_y_mm       = 40.0;
    in.axis_dir            = gp_Dir(0, 0, -1);
    in.nut_od_mm           = 25.0;
    in.nut_l_mm            = 20.0;
    in.mounting_pattern    = "square4";

    auto out = skill::ball_screw_nut_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vBefore = volumeOf(stock->shape());
    const double vAfter  = volumeOf(out.workpiece->shape());
    EXPECT_LT(vAfter, vBefore);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());

    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 6);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillBallScrewNutPocket, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 50.0);

    skill::ball_screw_nut_pocket::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.nut_od_mm        = -1.0;   // bad
    in.nut_l_mm         = 20.0;
    in.mounting_pattern = "square4";

    auto r = skill::ball_screw_nut_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);

    EXPECT_THROW(skill::ball_screw_nut_pocket::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind ───────────────────────────────────
TEST(SkillBallScrewNutPocket, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 50.0);

    skill::ball_screw_nut_pocket::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 40.0;
    in.position_y_mm    = 40.0;
    in.nut_od_mm        = 25.0;
    in.nut_l_mm         = 20.0;
    in.mounting_pattern = "square4";

    auto out = skill::ball_screw_nut_pocket::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("ball_screw_nut_pocket"));
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("ball_screw_nut_pocket"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillBallScrewNutPocket, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 50.0);

    skill::ball_screw_nut_pocket::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 40.0;
    in.position_y_mm    = 40.0;
    in.nut_od_mm        = 25.0;
    in.nut_l_mm         = 20.0;
    in.mounting_pattern = "square4";

    auto out = skill::ball_screw_nut_pocket::apply(*stock, in);

    auto cands = skill::ball_screw_nut_pocket::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].skill_id, std::string("ball_screw_nut_pocket"));
    EXPECT_DOUBLE_EQ(cands[0].recovered_params.at("nut_od_mm").get<double>(), 25.0);
}

// ─── 5. slip_dia_mm = nut_od + 2 × slip_clearance (geometry invariant) ────
TEST(SkillBallScrewNutPocket, SlipDiaMatchesNutOdPlusClearance)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 50.0);

    skill::ball_screw_nut_pocket::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 40.0;
    in.position_y_mm    = 40.0;
    in.nut_od_mm        = 30.0;
    in.nut_l_mm         = 20.0;
    in.mounting_pattern = "diamond4";

    auto out = skill::ball_screw_nut_pocket::apply(*stock, in);

    const double slipDia       = out.signature.pattern.at("slip_dia_mm").get<double>();
    const double slipClearance = out.signature.pattern.at("slip_clearance_mm").get<double>();
    const double nutOd         = out.signature.pattern.at("nut_od_mm").get<double>();
    EXPECT_NEAR(slipDia, nutOd + 2.0 * slipClearance, 1e-9);
    EXPECT_EQ(out.signature.pattern.at("mounting_pattern").get<std::string>(),
              std::string("diamond4"));
}
