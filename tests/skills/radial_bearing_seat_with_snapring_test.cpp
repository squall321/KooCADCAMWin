// @lat: [[process/test-strategy#compound bearing seat]]
//
// radial_bearing_seat_with_snapring — compound: outer bore + shoulder step
// + DIN 471 snap-ring groove + 45° lead chamfer.
//
// Test cases:
//   1. apply removes a coherent volume and yields a multi-face workpiece.
//   2. validate rejects an out-of-range snap-ring bore (DIN 471 < 10 mm).
//   3. signature records is_compound + subfeature_count >= 2.
//   4. recognize replays from metadata at confidence 1.0.
//   5. DIN 471 snap-ring thickness table matches published values.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/radial_bearing_seat_with_snapring.hpp"

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
TEST(SkillRadialBearingSeatSnapring, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::radial_bearing_seat_with_snapring::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.outer_dia_mm  = 22.0;
    in.inner_dia_mm  = 15.0;
    in.depth_mm      = 12.0;
    in.snap_ring_std = "DIN471";

    const double v0 = volumeOf(stock->shape());
    const int    f0 = stock->faceCount();

    auto out = skill::radial_bearing_seat_with_snapring::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    const int    f1 = out.workpiece->faceCount();

    // Volume removed = outer bore (depth) + extra shoulder column + groove
    // ring.  Approximate lower & upper bounds.
    const double outerVol = M_PI * 11.0 * 11.0 * 12.0;   // ~4560
    const double removed  = v0 - v1;
    EXPECT_GT(removed, outerVol * 0.95);
    EXPECT_LT(removed, outerVol * 2.5);   // < 2.5× outer bore (safety upper)

    // Compound feature should increase the face count noticeably (multiple
    // cylindrical + annular bands + chamfer cone).
    EXPECT_GT(f1, f0 + 2);
}

// ─── 2. Validate rejects an outer_dia outside DIN 471 range ───────────────
TEST(SkillRadialBearingSeatSnapring, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::radial_bearing_seat_with_snapring::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.outer_dia_mm  = 8.0;     // < 10 mm → outside DIN 471 range
    in.inner_dia_mm  = 5.0;
    in.depth_mm      = 6.0;

    auto r = skill::radial_bearing_seat_with_snapring::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DIN471-RANGE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::radial_bearing_seat_with_snapring::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind ──────────────────────────────────
TEST(SkillRadialBearingSeatSnapring, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::radial_bearing_seat_with_snapring::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.outer_dia_mm  = 22.0;
    in.inner_dia_mm  = 15.0;
    in.depth_mm      = 12.0;
    auto out = skill::radial_bearing_seat_with_snapring::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("radial_bearing_seat_with_snapring"));
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("radial_bearing_seat_with_snapring"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillRadialBearingSeatSnapring, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::radial_bearing_seat_with_snapring::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.outer_dia_mm  = 22.0;
    in.inner_dia_mm  = 15.0;
    in.depth_mm      = 12.0;
    auto out = skill::radial_bearing_seat_with_snapring::apply(*stock, in);

    auto cands = skill::radial_bearing_seat_with_snapring::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["outer_dia_mm"].get<double>(),
                22.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["inner_dia_mm"].get<double>(),
                15.0, 1e-6);
}

// ─── 5. DIN 471 snap-ring thickness table sanity check ────────────────────
TEST(SkillRadialBearingSeatSnapring, SnapRingThicknessTableMatchesDIN471)
{
    using skill::radial_bearing_seat_with_snapring::snapRingThicknessFor;

    EXPECT_NEAR(snapRingThicknessFor(15.0), 1.0, 1e-9);
    EXPECT_NEAR(snapRingThicknessFor(22.0), 1.2, 1e-9);
    EXPECT_NEAR(snapRingThicknessFor(40.0), 1.5, 1e-9);
    EXPECT_NEAR(snapRingThicknessFor(60.0), 2.0, 1e-9);
    // Out-of-range yields zero (no thickness).
    EXPECT_NEAR(snapRingThicknessFor(5.0), 0.0, 1e-9);
    EXPECT_NEAR(snapRingThicknessFor(120.0), 0.0, 1e-9);

    // Groove diameter must be greater than the bore (DIN 471 d2 > d1).
    const double od = 22.0;
    const double t  = snapRingThicknessFor(od);
    const double grooveOD = od + 0.7 * t;
    EXPECT_GT(grooveOD, od);
}
