// @lat: [[engine/reverse-route#modify-in-place]]
//
// Locks in the "recover a hole from a foreign STEP, change its diameter ON THE
// ORIGINAL geometry, verify by re-recognition" capability that the koo_modify
// tool performs — as a deterministic regression test (no real-file dependency).
//
//   synth cuboid + Ø10 hole  ->  STEP round-trip (strip metadata)
//   ->  recognize (measured Ø10)  ->  ENLARGE in place (coaxial cut on the
//       imported shape, preserving the rest)  ->  STEP round-trip
//   ->  recognize  ->  assert the hole is now Ø14 at the same position.

#include <gtest/gtest.h>

#include "io/StepIO.hpp"
#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"

#include "engine/primitives/Tools.hpp"
#include "engine/primitives/Cuts.hpp"

#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>

using namespace koocadcam;
namespace pr = koocadcam::engine::prim;

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

// The single recovered drill_hole nearest (tx,ty); dia<0 if none.
struct Hole { double x, y, dia; gp_Dir axis; };
Hole recoverHoleNear(const skill::Workpiece& wp, double tx, double ty)
{
    Hole best{ 0, 0, -1, gp_Dir(0,0,-1) };
    double bestD = 5.0;
    for (const auto& c : re::analyze(wp)) {
        if (c.skill_id != "drill_hole" || c.confidence < 0.7) continue;
        const auto& p = c.recovered_params;
        const double x = p.value("position_x_mm", 0.0);
        const double y = p.value("position_y_mm", 0.0);
        const double d = std::hypot(x - tx, y - ty);
        if (d <= bestD) {
            bestD = d;
            gp_Dir ax(0,0,-1);
            if (p.contains("axis_dir") && p["axis_dir"].is_array() && p["axis_dir"].size() == 3)
                ax = gp_Dir(p["axis_dir"][0].get<double>(),
                            p["axis_dir"][1].get<double>(),
                            p["axis_dir"][2].get<double>());
            best = { x, y, p.value("diameter_mm", 0.0), ax };
        }
    }
    return best;
}

}  // namespace

// ─── Recover a foreign hole, enlarge it in place, re-verify the new size ──
TEST(ModifyInPlace, EnlargeRecoveredHoleAndReverify)
{
    constexpr double kOldDia = 10.0, kNewDia = 14.0;
    constexpr double kCx = 30.0, kCy = 30.0, kThk = 20.0;

    // Synthesize a cuboid with a Ø10 through-hole.
    auto stock = skill::createCuboidStock(60.0, 60.0, kThk);
    skill::drill_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = kCx; in.position_y_mm = kCy;
    in.axis_dir = gp_Dir(0, 0, -1);
    in.diameter_mm = kOldDia; in.depth_mm = 0.0; in.through_hole = true;
    const TopoDS_Shape s0 = skill::drill_hole::apply(*stock, in).workpiece->shape();

    // STEP round-trip → recover the Ø10 hole (measured, geometric path).
    int feat = -1;
    skill::Workpiece foreign = stepRoundTrip(s0, feat);
    ASSERT_EQ(feat, 0) << "STEP must strip metadata (geometric recovery only)";
    const Hole h0 = recoverHoleNear(foreign, kCx, kCy);
    ASSERT_GT(h0.dia, 0.0) << "original hole not recovered";
    EXPECT_NEAR(h0.dia, kOldDia, 0.06);

    // ENLARGE in place: cut a larger co-axial cylinder on the IMPORTED shape,
    // exactly as koo_modify does — the rest of the part is preserved.
    const double diag = std::sqrt(60.0*60.0 + 60.0*60.0 + kThk*kThk);
    const gp_Pnt start(h0.x - h0.axis.X()*0.1*diag,
                       h0.y - h0.axis.Y()*0.1*diag,
                       kThk - h0.axis.Z()*0.1*diag);
    const TopoDS_Shape s1 =
        pr::cut(foreign.shape(), pr::cylinder(gp_Ax2(start, h0.axis), kNewDia/2.0, diag*1.2));

    // STEP round-trip → the hole is now Ø14 at the same position.
    int feat2 = -1;
    skill::Workpiece foreign2 = stepRoundTrip(s1, feat2);
    ASSERT_EQ(feat2, 0);
    const Hole h1 = recoverHoleNear(foreign2, kCx, kCy);
    ASSERT_GT(h1.dia, 0.0) << "enlarged hole not recovered";
    EXPECT_NEAR(h1.dia, kNewDia, 0.06) << "hole must measure Ø14 after in-place enlarge";
    EXPECT_NEAR(h1.x, kCx, 0.15);
    EXPECT_NEAR(h1.y, kCy, 0.15);
}
