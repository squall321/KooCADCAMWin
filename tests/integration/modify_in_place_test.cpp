// @lat: [[engine/reverse-route#modify-in-place]]
//
// Locks the "recover a hole from a foreign STEP, edit it ON THE ORIGINAL
// geometry, verify by re-recognition" capability — as deterministic
// regression tests (no real-file dependency).
//
// All edits run through edit::editHole (plan B4.1): defeature (remove the
// hole's faces + heal the solid, restoring material) then recut.  That makes
// ENLARGE, SHRINK and MOVE the same material-removal operation — the old
// fuse-based shrink/move path (annular ring + far_center heuristic, the
// source of the reverted 1046 mm fill-tube failure) is gone.
//
//   synth cuboid + hole  ->  STEP round-trip (strip metadata)
//   ->  recognize (measured, geometric path)  ->  editHole  ->  STEP
//   round-trip  ->  recognize  ->  assert the edited hole.

#include <gtest/gtest.h>

#include "edit/FeatureEditor.hpp"
#include "io/StepIO.hpp"
#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>

using namespace koocadcam;

namespace {

// STEP round-trip → fresh Workpiece (no embedded feature history → geometric
// recovery only).  featuresAfter must be 0 (proves non-circular recognition).
skill::Workpiece stepRoundTrip(const TopoDS_Shape& shape, int& featuresAfter)
{
    namespace fs = std::filesystem;
    const fs::path p = fs::temp_directory_path() /
                       ("koo_modinplace_" + std::to_string(::rand()) + ".step");
    std::string err;
    EXPECT_TRUE(io::StepIO::write(shape, p, err)) << err;
    auto reim = io::StepIO::read(p, err);
    EXPECT_TRUE(reim.has_value()) << err;
    std::error_code ec; fs::remove(p, ec);
    skill::Workpiece wp(*reim);
    featuresAfter = static_cast<int>(wp.features().size());
    return wp;
}

// Recovered params of the drill_hole nearest (tx,ty); empty json if none
// within 5 mm.
nlohmann::json recoverHoleParamsNear(const skill::Workpiece& wp,
                                     double tx, double ty)
{
    nlohmann::json best;
    double bestD = 5.0;
    for (const auto& c : re::dedupe(re::analyze(wp))) {
        if (c.skill_id != "drill_hole" || c.confidence < 0.7) continue;
        const auto& p = c.recovered_params;
        const double x = p.value("position_x_mm", 0.0);
        const double y = p.value("position_y_mm", 0.0);
        const double d = std::hypot(x - tx, y - ty);
        if (d <= bestD) { bestD = d; best = p; }
    }
    return best;
}

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps g;
    BRepGProp::VolumeProperties(s, g);
    return g.Mass();
}

// Synthesize a 60x60x20 cuboid with one drilled hole and round-trip it so
// only geometric recovery is possible.
skill::Workpiece foreignPartWithHole(double cx, double cy, double dia,
                                     double depth, bool through)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::drill_hole::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = cx;
    in.position_y_mm = cy;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.diameter_mm   = dia;
    in.depth_mm      = through ? 0.0 : depth;
    in.through_hole  = through;
    const TopoDS_Shape s0 =
        skill::drill_hole::apply(*stock, in).workpiece->shape();
    int feat = -1;
    skill::Workpiece wp = stepRoundTrip(s0, feat);
    EXPECT_EQ(feat, 0) << "STEP must strip metadata (geometric recovery only)";
    return wp;
}

}  // namespace

// ─── 1. ENLARGE: Ø10 → Ø14 at the same position ───────────────────────────
TEST(ModifyInPlace, EnlargeRecoveredHoleAndReverify)
{
    skill::Workpiece foreign = foreignPartWithHole(30.0, 30.0, 10.0, 0.0, true);
    const nlohmann::json rec = recoverHoleParamsNear(foreign, 30.0, 30.0);
    ASSERT_FALSE(rec.empty()) << "original hole not recovered";
    EXPECT_NEAR(rec.value("diameter_mm", 0.0), 10.0, 0.06);

    auto oldHole = edit::holeSpecFromRecovered(rec);
    ASSERT_TRUE(oldHole.has_value());
    edit::HoleSpec newHole = *oldHole;
    newHole.diameter_mm    = 14.0;

    std::string err;
    const TopoDS_Shape s1 =
        edit::editHole(foreign.shape(), *oldHole, newHole, err);
    ASSERT_FALSE(s1.IsNull()) << err;

    int feat2 = -1;
    skill::Workpiece foreign2 = stepRoundTrip(s1, feat2);
    ASSERT_EQ(feat2, 0);
    const nlohmann::json r2 = recoverHoleParamsNear(foreign2, 30.0, 30.0);
    ASSERT_FALSE(r2.empty()) << "enlarged hole not recovered";
    EXPECT_NEAR(r2.value("diameter_mm", 0.0), 14.0, 0.06);
    EXPECT_NEAR(r2.value("position_x_mm", 0.0), 30.0, 0.15);
    EXPECT_NEAR(r2.value("position_y_mm", 0.0), 30.0, 0.15);
}

// ─── 2. SHRINK: Ø10 → Ø6 — the case the fuse path could never lock ────────
TEST(ModifyInPlace, ShrinkRecoveredHoleAndReverify)
{
    skill::Workpiece foreign = foreignPartWithHole(30.0, 30.0, 10.0, 0.0, true);
    const nlohmann::json rec = recoverHoleParamsNear(foreign, 30.0, 30.0);
    ASSERT_FALSE(rec.empty());

    auto oldHole = edit::holeSpecFromRecovered(rec);
    ASSERT_TRUE(oldHole.has_value());
    edit::HoleSpec newHole = *oldHole;
    newHole.diameter_mm    = 6.0;

    std::string err;
    const TopoDS_Shape s1 =
        edit::editHole(foreign.shape(), *oldHole, newHole, err);
    ASSERT_FALSE(s1.IsNull()) << err;

    // Shrinking restores material: π(5²−3²)·20 ≈ 1005 mm³.
    const double expect = M_PI * (25.0 - 9.0) * 20.0;
    const double dV = volumeOf(s1) - volumeOf(foreign.shape());
    EXPECT_NEAR(dV, expect, expect * 0.05)
        << "shrink must ADD back the annulus volume";

    int feat2 = -1;
    skill::Workpiece foreign2 = stepRoundTrip(s1, feat2);
    ASSERT_EQ(feat2, 0);
    const nlohmann::json r2 = recoverHoleParamsNear(foreign2, 30.0, 30.0);
    ASSERT_FALSE(r2.empty()) << "shrunk hole not recovered";
    EXPECT_NEAR(r2.value("diameter_mm", 0.0), 6.0, 0.06);
}

// ─── 3. MOVE: Ø10 at (30,30) → (42,22) — previously reverted as infeasible ─
TEST(ModifyInPlace, MoveRecoveredHoleAndReverify)
{
    skill::Workpiece foreign = foreignPartWithHole(30.0, 30.0, 10.0, 0.0, true);
    const nlohmann::json rec = recoverHoleParamsNear(foreign, 30.0, 30.0);
    ASSERT_FALSE(rec.empty());

    auto oldHole = edit::holeSpecFromRecovered(rec);
    ASSERT_TRUE(oldHole.has_value());
    edit::HoleSpec newHole = *oldHole;
    newHole.entry = gp_Pnt(42.0, 22.0, oldHole->entry.Z());

    std::string err;
    const TopoDS_Shape s1 =
        edit::editHole(foreign.shape(), *oldHole, newHole, err);
    ASSERT_FALSE(s1.IsNull()) << err;

    // Same hole, different place → volume unchanged within Boolean noise.
    EXPECT_NEAR(volumeOf(s1), volumeOf(foreign.shape()),
                volumeOf(foreign.shape()) * 0.005);

    int feat2 = -1;
    skill::Workpiece foreign2 = stepRoundTrip(s1, feat2);
    ASSERT_EQ(feat2, 0);

    const nlohmann::json atNew = recoverHoleParamsNear(foreign2, 42.0, 22.0);
    ASSERT_FALSE(atNew.empty()) << "moved hole not recovered at new position";
    EXPECT_NEAR(atNew.value("diameter_mm", 0.0), 10.0, 0.06);
    EXPECT_NEAR(atNew.value("position_x_mm", 0.0), 42.0, 0.15);
    EXPECT_NEAR(atNew.value("position_y_mm", 0.0), 22.0, 0.15);

    const nlohmann::json atOld = recoverHoleParamsNear(foreign2, 30.0, 30.0);
    EXPECT_TRUE(atOld.empty())
        << "the old position must be HEALED — no hole may remain there";
}

// ─── 4. BLIND-hole shrink: defeature must also remove the bottom face ─────
TEST(ModifyInPlace, ShrinkBlindHoleAndReverify)
{
    skill::Workpiece foreign =
        foreignPartWithHole(30.0, 30.0, 10.0, 12.0, /*through=*/false);
    const nlohmann::json rec = recoverHoleParamsNear(foreign, 30.0, 30.0);
    ASSERT_FALSE(rec.empty());
    EXPECT_NEAR(rec.value("depth_mm", 0.0), 12.0, 0.2);

    auto oldHole = edit::holeSpecFromRecovered(rec);
    ASSERT_TRUE(oldHole.has_value());
    edit::HoleSpec newHole = *oldHole;
    newHole.diameter_mm    = 6.0;

    std::string err;
    const TopoDS_Shape s1 =
        edit::editHole(foreign.shape(), *oldHole, newHole, err);
    ASSERT_FALSE(s1.IsNull()) << err;

    int feat2 = -1;
    skill::Workpiece foreign2 = stepRoundTrip(s1, feat2);
    ASSERT_EQ(feat2, 0);
    const nlohmann::json r2 = recoverHoleParamsNear(foreign2, 30.0, 30.0);
    ASSERT_FALSE(r2.empty()) << "shrunk blind hole not recovered";
    EXPECT_NEAR(r2.value("diameter_mm", 0.0), 6.0, 0.06);
    EXPECT_NEAR(r2.value("depth_mm", 0.0), 12.0, 0.3);
}
