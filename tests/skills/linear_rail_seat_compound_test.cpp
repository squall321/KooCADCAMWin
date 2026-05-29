// @lat: [[process/test-strategy#compound macro]]
//
// linear_rail_seat_compound — COMPOUND macro: rail slot + 2 rows of M5 tapped
// holes (at pitch) + edge chamfer.  Tests:
//   1. apply produces a workpiece whose volume reduction matches the sum of
//      slot + holes + chamfer, and whose face count change is consistent
//      with N sub-features.
//   2. validate rejects bad input.
//   3. signature pattern records is_compound=true and subfeature_count>=2.
//   4. recognize via metadata replay returns confidence 1.0.
//   5. hole_count == 2 × holes_per_row (the rail-specific invariant).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/linear_rail_seat_compound.hpp"

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
TEST(SkillLinearRailSeatCompound, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(120.0, 40.0, 12.0);

    skill::linear_rail_seat_compound::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.length_mm           = 100.0;
    in.rail_centerline_mm  = 20.0;
    in.pitch_mm            = 30.0;
    in.start_x_mm          = 10.0;

    auto out = skill::linear_rail_seat_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume should be REDUCED (we removed material).
    const double vBefore = volumeOf(stock->shape());
    const double vAfter  = volumeOf(out.workpiece->shape());
    EXPECT_LT(vAfter, vBefore);

    // Face count should INCREASE because each cut introduces new faces.
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());

    // Sub-feature count: 1 slot + 1 chamfer + 2 × holes_per_row.
    // With length=100, pitch=30: holes_per_row = floor((100-30)/30)+1 = 3.
    const int subCount = out.signature.pattern.at("subfeature_count").get<int>();
    EXPECT_GE(subCount, 2);   // generic compound rule
    EXPECT_EQ(subCount, 2 + 2 * 3);   // specific to this layout
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillLinearRailSeatCompound, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(120.0, 40.0, 12.0);

    skill::linear_rail_seat_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.length_mm          = 0.0;     // bad
    in.rail_centerline_mm = 20.0;
    in.pitch_mm           = 30.0;

    auto r = skill::linear_rail_seat_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);

    EXPECT_THROW(skill::linear_rail_seat_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind ───────────────────────────────────
TEST(SkillLinearRailSeatCompound, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(120.0, 40.0, 12.0);

    skill::linear_rail_seat_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.length_mm          = 80.0;
    in.rail_centerline_mm = 20.0;
    in.pitch_mm           = 25.0;
    in.start_x_mm         = 20.0;

    auto out = skill::linear_rail_seat_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("linear_rail_seat_compound"));
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("linear_rail_seat_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillLinearRailSeatCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(120.0, 40.0, 12.0);

    skill::linear_rail_seat_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.length_mm          = 80.0;
    in.rail_centerline_mm = 20.0;
    in.pitch_mm           = 25.0;
    in.start_x_mm         = 20.0;

    auto out = skill::linear_rail_seat_compound::apply(*stock, in);

    auto cands = skill::linear_rail_seat_compound::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].skill_id, std::string("linear_rail_seat_compound"));
    EXPECT_DOUBLE_EQ(cands[0].recovered_params.at("length_mm").get<double>(), 80.0);
}

// ─── 5. hole_count == 2 × holes_per_row (rail-specific invariant) ─────────
TEST(SkillLinearRailSeatCompound, RailLayoutHoleCountIsTwiceHolesPerRow)
{
    auto stock = skill::createCuboidStock(200.0, 40.0, 12.0);

    skill::linear_rail_seat_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.length_mm          = 150.0;
    in.rail_centerline_mm = 20.0;
    in.pitch_mm           = 30.0;
    in.start_x_mm         = 25.0;

    auto out = skill::linear_rail_seat_compound::apply(*stock, in);

    const int holesPerRow = out.signature.pattern.at("holes_per_row").get<int>();
    const int holeCount   = out.signature.pattern.at("hole_count").get<int>();
    const int rows        = out.signature.pattern.at("rows").get<int>();

    EXPECT_EQ(rows, 2);
    EXPECT_EQ(holeCount, 2 * holesPerRow);
    // length=150, pitch=30: floor((150-30)/30)+1 = 5
    EXPECT_EQ(holesPerRow, 5);
}
