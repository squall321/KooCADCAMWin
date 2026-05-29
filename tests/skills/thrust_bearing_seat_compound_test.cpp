// @lat: [[process/test-strategy#compound bearing seat]]
//
// thrust_bearing_seat_compound — compound: face cut + raceway ring groove +
// lock-nut pilot + thrust shoulder.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/thrust_bearing_seat_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. Apply produces valid shape with multiple sub-features ─────────────
TEST(SkillThrustBearingSeatCompound, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::thrust_bearing_seat_compound::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.outer_dia_mm  = 24.0;
    in.inner_dia_mm  = 12.0;
    in.face_depth_mm = 1.0;
    in.seat_depth_mm = 6.0;

    const double v0 = volumeOf(stock->shape());
    const int    f0 = stock->faceCount();

    auto out = skill::thrust_bearing_seat_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    const int    f1 = out.workpiece->faceCount();

    // Lower bound: outer face cut + locknut pilot volume.
    const double faceVol    = M_PI * 12.0 * 12.0 * 1.0;
    const double pilotVol   = M_PI *  6.0 *  6.0 * 7.0;
    const double removed    = v0 - v1;
    EXPECT_GT(removed, (faceVol + pilotVol) * 0.85);
    EXPECT_LT(removed, (faceVol + pilotVol) * 1.5);

    // More than just the stock's 6 outer faces — sub-features add surfaces.
    EXPECT_GT(f1, f0);
}

// ─── 2. Validate rejects inverted (inner ≥ outer) ─────────────────────────
TEST(SkillThrustBearingSeatCompound, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::thrust_bearing_seat_compound::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.outer_dia_mm  = 15.0;
    in.inner_dia_mm  = 15.0;   // equal → no race width → DFM error
    in.face_depth_mm = 1.0;
    in.seat_depth_mm = 5.0;

    auto r = skill::thrust_bearing_seat_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SEAT-GEOM") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::thrust_bearing_seat_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature carries compound metadata ───────────────────────────────
TEST(SkillThrustBearingSeatCompound, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::thrust_bearing_seat_compound::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.outer_dia_mm  = 24.0;
    in.inner_dia_mm  = 12.0;
    in.face_depth_mm = 1.0;
    in.seat_depth_mm = 6.0;
    auto out = skill::thrust_bearing_seat_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("thrust_bearing_seat_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillThrustBearingSeatCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::thrust_bearing_seat_compound::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.outer_dia_mm  = 24.0;
    in.inner_dia_mm  = 12.0;
    in.face_depth_mm = 1.0;
    in.seat_depth_mm = 6.0;
    auto out = skill::thrust_bearing_seat_compound::apply(*stock, in);

    auto cands = skill::thrust_bearing_seat_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
}

// ─── 5. Raceway groove geometry derived per Timken thrust spec ────────────
TEST(SkillThrustBearingSeatCompound, RacewayGrooveSpecificDerivation)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::thrust_bearing_seat_compound::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.outer_dia_mm  = 30.0;
    in.inner_dia_mm  = 18.0;
    in.face_depth_mm = 1.0;
    in.seat_depth_mm = 5.0;
    auto out = skill::thrust_bearing_seat_compound::apply(*stock, in);

    const double race_width = 30.0 - 18.0;
    EXPECT_NEAR(out.signature.pattern.at("race_width_mm").get<double>(),
                race_width, 1e-9);
    // groove width = race_width / 6 (= 2.0), depth = race_width / 8 (= 1.5)
    EXPECT_NEAR(out.signature.pattern.at("raceway_groove_width_mm").get<double>(),
                race_width / 6.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern.at("raceway_groove_depth_mm").get<double>(),
                race_width / 8.0, 1e-9);
}
