// @lat: [[process/test-strategy#compound macro]]
//
// adjuster_screw_pocket_compound — 3-sub-feature compound macro:
//   pilot (fine-pitch tap) + lock-nut counterbore + scribe groove on rim.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/adjuster_screw_pocket_compound.hpp"

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

// ─── 1. apply produces a valid shape with measurable volume removed ───
TEST(SkillAdjusterScrewPocketCompound, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    const int faces0 = stock->faceCount();

    skill::adjuster_screw_pocket_compound::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.fine_thread      = "M5x0.5";   // pilot = 4.5 mm
    in.lock_nut_M       = "M5";       // af = 8.0 mm → seat dia 8.4 mm
    in.tap_depth_mm     = 6.0;
    in.nut_thickness_mm = 4.0;
    in.scribe_groove_width_mm = 0.3;
    in.scribe_groove_depth_mm = 0.15;

    const double v0 = volumeOf(stock->shape());
    auto out = skill::adjuster_screw_pocket_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // volume was removed

    // Multi-cut → face count should change (more than just a flat-top
    // box: at least pilot + seat sidewalls + scribe inner/outer walls).
    EXPECT_GT(out.workpiece->faceCount(), faces0);
}

// ─── 2. validate rejects bad input ────────────────────────────────────
TEST(SkillAdjusterScrewPocketCompound, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::adjuster_screw_pocket_compound::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.fine_thread      = "M99x9";    // unknown
    in.lock_nut_M       = "M5";
    in.tap_depth_mm     = 5.0;
    in.nut_thickness_mm = 4.0;

    auto r = skill::adjuster_screw_pocket_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ADJ-FINE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::adjuster_screw_pocket_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. signature records compound kind ───────────────────────────────
TEST(SkillAdjusterScrewPocketCompound, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::adjuster_screw_pocket_compound::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.fine_thread      = "M4x0.5";
    in.lock_nut_M       = "M4";
    in.tap_depth_mm     = 5.0;
    in.nut_thickness_mm = 3.0;

    auto out = skill::adjuster_screw_pocket_compound::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id,
              std::string("adjuster_screw_pocket_compound"));
    EXPECT_EQ(out.signature.pattern.at("kind"),
              std::string("adjuster_screw_pocket_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
    EXPECT_NEAR(out.signature.pattern.at("pilot_dia_mm").get<double>(), 3.5, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("pitch_mm").get<double>(),    0.5, 1e-6);
}

// ─── 4. recognize via metadata replay ─────────────────────────────────
TEST(SkillAdjusterScrewPocketCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::adjuster_screw_pocket_compound::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.fine_thread      = "M6x0.75";
    in.lock_nut_M       = "M6";
    in.tap_depth_mm     = 8.0;
    in.nut_thickness_mm = 5.0;

    auto out = skill::adjuster_screw_pocket_compound::apply(*stock, in);
    auto cands = skill::adjuster_screw_pocket_compound::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["fine_thread"], "M6x0.75");
    EXPECT_EQ(cands[0].recovered_params["lock_nut_M"],  "M6");
}

// ─── 5. SPECIFIC: lock-nut seat dia > pilot dia (DIN 934 geometry) ────
TEST(SkillAdjusterScrewPocketCompound, SeatDiameterExceedsPilotDiameter)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::adjuster_screw_pocket_compound::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.fine_thread      = "M3x0.35";   // pilot = 2.65 mm
    in.lock_nut_M       = "M3";        // af = 5.5 → seat 5.9 mm
    in.tap_depth_mm     = 3.0;
    in.nut_thickness_mm = 2.5;

    auto out = skill::adjuster_screw_pocket_compound::apply(*stock, in);
    const double pilotDia = out.signature.pattern.at("pilot_dia_mm").get<double>();
    const double seatDia  = out.signature.pattern.at("seat_dia_mm").get<double>();
    EXPECT_GT(seatDia, pilotDia);
    // Specifically: seat = AF + 0.4 mm clearance for M3
    EXPECT_NEAR(seatDia, 5.9, 1e-6);
    // Scribe groove inner radius > seat radius (groove sits OUTSIDE seat)
    const double scribeInnerR = out.signature.pattern.at("scribe_inner_r_mm").get<double>();
    EXPECT_GT(scribeInnerR, seatDia / 2.0);
}
