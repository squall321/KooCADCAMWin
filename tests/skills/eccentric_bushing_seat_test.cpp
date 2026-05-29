// @lat: [[process/test-strategy#compound macro]]
//
// eccentric_bushing_seat — 4 sub-features:
//   bore + 2 oblong arc slots + retention chamfer.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/eccentric_bushing_seat.hpp"

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

// ─── 1. apply produces a valid shape with multiple sub-features ───────
TEST(SkillEccentricBushingSeat, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    const int faces0 = stock->faceCount();

    skill::eccentric_bushing_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.bore_od_mm     = 16.0;
    in.bore_depth_mm  = 8.0;
    in.slot_arc_deg   = 60.0;
    in.slot_width_mm  = 2.0;
    in.slot_depth_mm  = 2.0;
    in.chamfer_size_mm = 0.5;

    const double v0 = volumeOf(stock->shape());
    auto out = skill::eccentric_bushing_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Volume removed > bore volume alone (slots add to total removal).
    const double boreOnly = M_PI * 8.0 * 8.0 * 8.0;
    EXPECT_GT(v0 - v1, boreOnly);

    EXPECT_GT(out.workpiece->faceCount(), faces0);
}

// ─── 2. validate rejects bad input ────────────────────────────────────
TEST(SkillEccentricBushingSeat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::eccentric_bushing_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.bore_od_mm     = 16.0;
    in.bore_depth_mm  = 8.0;
    in.slot_arc_deg   = 200.0;     // > 120° → DFM-EB-ARC error
    in.slot_width_mm  = 2.0;
    in.slot_depth_mm  = 2.0;

    auto r = skill::eccentric_bushing_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-EB-ARC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::eccentric_bushing_seat::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. signature records compound kind ───────────────────────────────
TEST(SkillEccentricBushingSeat, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::eccentric_bushing_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.bore_od_mm     = 14.0;
    in.bore_depth_mm  = 7.0;
    in.slot_arc_deg   = 45.0;
    in.slot_width_mm  = 1.6;
    in.slot_depth_mm  = 1.6;

    auto out = skill::eccentric_bushing_seat::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("eccentric_bushing_seat"));
    EXPECT_EQ(out.signature.pattern.at("kind"),
              std::string("eccentric_bushing_seat"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
    EXPECT_NEAR(out.signature.pattern.at("slot_arc_deg").get<double>(), 45.0, 1e-9);
    EXPECT_NEAR(out.signature.params.at("bore_od_mm").get<double>(), 14.0, 1e-9);
}

// ─── 4. recognize via metadata replay ─────────────────────────────────
TEST(SkillEccentricBushingSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::eccentric_bushing_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.bore_od_mm     = 12.0;
    in.bore_depth_mm  = 6.0;
    in.slot_arc_deg   = 30.0;
    in.slot_width_mm  = 1.5;
    in.slot_depth_mm  = 1.5;

    auto out = skill::eccentric_bushing_seat::apply(*stock, in);
    auto cands = skill::eccentric_bushing_seat::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["bore_od_mm"].get<double>(),
                12.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["slot_arc_deg"].get<double>(),
                30.0, 1e-9);
}

// ─── 5. SPECIFIC: slot chord = 2 × r × sin(arc/2) (geometric identity) ─
TEST(SkillEccentricBushingSeat, SlotChordMatchesArcGeometry)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::eccentric_bushing_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.bore_od_mm     = 20.0;     // r = 10
    in.bore_depth_mm  = 5.0;
    in.slot_arc_deg   = 60.0;     // chord = 2 × 10 × sin 30° = 10
    in.slot_width_mm  = 2.0;
    in.slot_depth_mm  = 1.5;

    auto out = skill::eccentric_bushing_seat::apply(*stock, in);
    const double chord = out.signature.pattern.at("slot_chord_mm").get<double>();
    EXPECT_NEAR(chord, 10.0, 1e-6);

    // Two slot mid-angles must be 180° apart (anti-symmetric).
    const auto angles = out.signature.pattern.at("slot_angles_deg");
    ASSERT_EQ(angles.size(), 2u);
    EXPECT_NEAR(std::abs(angles[1].get<double>() - angles[0].get<double>()),
                180.0, 1e-9);
}
