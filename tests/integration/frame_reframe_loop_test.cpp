// @lat: [[engine/reverse-route#reframe-loop]]
//
// frame_reframe_loop_test — the original KooCADCAM vision, proven in
// miniature with MEASURED dimensions through a real STEP round-trip.
//
// The product goal is: "given an existing frame + a new parts layout,
// automatically regenerate the frame's machining."  This test exercises
// that loop end-to-end on a bolt-pattern frame plate and verifies every
// stage by MEASURING geometry recovered from a metadata-stripped STEP file
// — never by replaying the in-memory FeatureSignature history.
//
//   1. SYNTHESIZE   a frame plate (120x80x12) with a 4-hole bolt rectangle
//                   via a hand-authored ProcessPlan -> shape S1.
//   2. STEP ROUND-TRIP  StepIO::write(S1) -> read -> Workpiece reim.
//                   reim.features() is EMPTY — the signature history is gone,
//                   so any recovery below is GEOMETRIC, not circular replay.
//   3. RECOGNIZE    re::analyze(reim) -> drill_hole candidates whose
//                   diameter_mm and (x,y) are MEASURED from the cylinders.
//                   Asserted against the authored values to +/- tolerance.
//   4. REFRAME      simulate the "parts moved" event: the PCB the holes
//                   anchor to shifts +15 mm in X.  Build the adapted plan by
//                   shifting each recovered hole's position_x_mm by +15
//                   (deterministic adapt; the LLM bridge is the production
//                   strategy, but the test must be reproducible).
//   5. RE-EXECUTE   run the adapted plan on FRESH stock -> shape S2.
//   6. VERIFY       STEP round-trip S2, recognize, and assert (measured)
//                   that the 4 holes now sit at the shifted X and NONE remain
//                   at the original X.  Plus volume conservation.

#include <gtest/gtest.h>

#include "io/StepIO.hpp"
#include "parts/DatumGraph.hpp"
#include "parts/Part.hpp"
#include "parts/PartsLayout.hpp"
#include "process/Executor.hpp"
#include "process/ProcessPlan.hpp"
#include "process/StepInvocation.hpp"
#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;
using nlohmann::json;

namespace {

constexpr double kPlateX = 120.0, kPlateY = 80.0, kPlateZ = 12.0;
constexpr double kHoleDia = 6.0;

// The authored bolt rectangle (entry-face X,Y on the top face).
struct XY { double x, y; };
const std::vector<XY> kHoles = { {30, 20}, {90, 20}, {30, 60}, {90, 60} };

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

// Build a frame plate plan: 4 through-holes at the given centres.
process::ProcessPlan buildFramePlan(const std::vector<XY>& holes)
{
    process::ProcessPlan plan;
    for (const auto& h : holes) {
        process::StepInvocation s;
        s.skill_id = "drill_hole";
        s.params = {
            { "entry_face",    "top" },
            { "position_x_mm", h.x },
            { "position_y_mm", h.y },
            { "diameter_mm",   kHoleDia },
            { "depth_mm",      0.0 },
            { "through_hole",  true },
        };
        plan.append(s);
    }
    return plan;
}

// Run a plan on fresh stock and return the resulting shape.
TopoDS_Shape executeOnFreshStock(const process::ProcessPlan& plan)
{
    auto stock = skill::createCuboidStock(kPlateX, kPlateY, kPlateZ);
    auto res = process::Executor::execute(plan, stock);
    EXPECT_TRUE(res.ok()) << (res.errors.empty() ? "" : res.errors.front());
    return res.workpiece->shape();
}

// Strip metadata via a real STEP round-trip and return a fresh Workpiece.
// `featuresAfter` reports how many FeatureSignatures survive (must be 0).
skill::Workpiece stepRoundTrip(const TopoDS_Shape& shape, int& featuresAfter)
{
    namespace fs = std::filesystem;
    const fs::path p = fs::temp_directory_path() /
                       ("koo_reframe_" + std::to_string(::rand()) + ".step");
    std::string err;
    EXPECT_TRUE(io::StepIO::write(shape, p, err)) << err;
    auto reimShape = io::StepIO::read(p, err);
    EXPECT_TRUE(reimShape.has_value()) << err;
    std::error_code ec; fs::remove(p, ec);
    skill::Workpiece reim(*reimShape);
    featuresAfter = static_cast<int>(reim.features().size());
    return reim;
}

// Recover all geometric drill_hole candidates (measured x,y,dia) above conf.
struct RecHole { double x, y, dia; };
std::vector<RecHole> recoverHoles(const skill::Workpiece& wp, double minConf = 0.7)
{
    std::vector<RecHole> out;
    for (const auto& c : re::analyze(wp)) {
        if (c.skill_id != "drill_hole" || c.confidence < minConf) continue;
        const auto& rp = c.recovered_params;
        out.push_back({ rp.value("position_x_mm", 0.0),
                        rp.value("position_y_mm", 0.0),
                        rp.value("diameter_mm",   0.0) });
    }
    std::sort(out.begin(), out.end(), [](const RecHole& a, const RecHole& b) {
        if (std::abs(a.x - b.x) > 1e-6) return a.x < b.x;
        return a.y < b.y;
    });
    return out;
}

// Nearest recovered hole to (tx,ty); dia<0 if none within tol.
RecHole findNear(const std::vector<RecHole>& hs, double tx, double ty, double tol)
{
    RecHole best{ 0.0, 0.0, -1.0 };
    double bestD = tol * tol;
    for (const auto& h : hs) {
        const double d = (h.x - tx) * (h.x - tx) + (h.y - ty) * (h.y - ty);
        if (d <= bestD) { bestD = d; best = h; }
    }
    return best;
}

// Match a measured hole set against an expected XY set within tol (both ways).
::testing::AssertionResult holesMatch(const std::vector<RecHole>& got,
                                      const std::vector<XY>& want,
                                      double posTol)
{
    if (got.size() != want.size())
        return ::testing::AssertionFailure()
               << "hole count " << got.size() << " != expected " << want.size();
    for (const auto& w : want) {
        bool found = false;
        for (const auto& g : got)
            if (std::abs(g.x - w.x) <= posTol && std::abs(g.y - w.y) <= posTol) {
                found = true; break;
            }
        if (!found)
            return ::testing::AssertionFailure()
                   << "no recovered hole near expected (" << w.x << "," << w.y << ")";
    }
    return ::testing::AssertionSuccess();
}

}  // namespace

// ─── 1. Geometric recovery measures the REAL hole dimensions ──────────────
TEST(FrameReframeLoop, RecoverHolesMeasuredFromForeignStep)
{
    const TopoDS_Shape s1 = executeOnFreshStock(buildFramePlan(kHoles));

    int featuresAfter = -1;
    skill::Workpiece reim = stepRoundTrip(s1, featuresAfter);

    // HONESTY GATE: a STEP file carries no FeatureSignature, so recovery
    // below cannot be metadata replay — it is geometric measurement.
    EXPECT_EQ(featuresAfter, 0)
        << "re-imported STEP must have zero embedded features (else recovery "
           "would be circular metadata replay, not real recognition)";

    const std::vector<RecHole> holes = recoverHoles(reim);
    ASSERT_EQ(holes.size(), kHoles.size())
        << "expected 4 drill_hole features recovered geometrically";

    // Measured diameter == authored 6.0 mm within STEP+recovery tolerance.
    for (const auto& h : holes)
        EXPECT_NEAR(h.dia, kHoleDia, 0.06)
            << "recovered hole diameter must match the machined 6.0 mm";

    // Measured positions == the authored bolt rectangle.
    EXPECT_TRUE(holesMatch(holes, kHoles, 0.15));
}

// A PCB part whose AABB centre is (cx, 40, 6); its 4 mounting holes are the
// frame's 4 drills.  Used to drive the datum dependency graph.
parts::PartsLayout pcbLayout(double cx)
{
    parts::PartsLayout L;
    parts::Part pcb;
    pcb.id   = "pcb";
    pcb.xMin = cx - 40.0; pcb.xMax = cx + 40.0;   // centre x = cx
    pcb.yMin = 10.0;      pcb.yMax = 70.0;        // centre y = 40
    pcb.zMin = 0.0;       pcb.zMax = 12.0;        // centre z = 6
    L.addPart(pcb);
    return L;
}

// ─── 2. Full reframe loop, DATUM-DRIVEN: PCB moves +15mm X -> holes follow ─
//
// The adapt step is NOT a hardcoded shift — it runs the real dependency graph:
// reframePlanForMoves links each hole to the PCB (point↔AABB-centre within
// tolerance), diffs the layouts, and shifts every dependent step by the PCB's
// centre delta.  Verified by measuring the re-machined holes.
TEST(FrameReframeLoop, ReframeShiftsHolesByPartMove)
{
    constexpr double kDeltaX = 15.0;

    // Original frame -> STEP -> recover (geometric).
    const TopoDS_Shape s1 = executeOnFreshStock(buildFramePlan(kHoles));
    int feat = -1;
    skill::Workpiece reim = stepRoundTrip(s1, feat);
    ASSERT_EQ(feat, 0);
    const std::vector<RecHole> recovered = recoverHoles(reim);
    ASSERT_EQ(recovered.size(), kHoles.size());

    // Rebuild a plan from the recovered (measured) hole params.
    std::vector<XY> recXY;
    for (const auto& h : recovered) recXY.push_back({ h.x, h.y });
    const process::ProcessPlan recoveredPlan = buildFramePlan(recXY);

    // DATUM-DRIVEN ADAPT: the PCB moved +15mm X.  The PCB's AABB CONTAINS all
    // four mounting holes, so a tight 1mm margin anchors every hole to it (no
    // need to inflate the tolerance to reach a far-off part centre) and
    // reframePlanForMoves shifts them by the part delta.
    const parts::PartsLayout before = pcbLayout(60.0);
    const parts::PartsLayout after  = pcbLayout(60.0 + kDeltaX);
    const process::ProcessPlan adapted =
        parts::reframePlanForMoves(recoveredPlan, before, after, 1.0);

    // RE-EXECUTE on fresh stock, then verify by measuring the new STEP.
    const TopoDS_Shape s2 = executeOnFreshStock(adapted);
    int feat2 = -1;
    skill::Workpiece reim2 = stepRoundTrip(s2, feat2);
    ASSERT_EQ(feat2, 0);
    const std::vector<RecHole> reframed = recoverHoles(reim2);
    ASSERT_EQ(reframed.size(), kHoles.size());

    // Expected shifted bolt rectangle.
    std::vector<XY> want;
    for (const auto& h : kHoles) want.push_back({ h.x + kDeltaX, h.y });
    EXPECT_TRUE(holesMatch(reframed, want, 0.15))
        << "after datum-driven reframe, holes must follow the PCB part";

    // And NONE remain at an original X column (30 or 90).
    for (const auto& r : reframed) {
        EXPECT_GT(std::abs(r.x - 30.0), 1.0);
        EXPECT_GT(std::abs(r.x - 90.0), 1.0);
    }
}

// ─── 4. Compound feature: counterbore seat+pilot recovered by MEASUREMENT ─
//
// A counterbore is two coaxial cylinders (a wide shallow seat + a narrow deep
// pilot).  Synthesize one on the plate, strip metadata via STEP, and assert
// the recognizer recovers BOTH measured diameters — proving the loop handles
// compound features, not just single holes.
TEST(FrameReframeLoop, RecoverCounterboreMeasured)
{
    constexpr double kSeatDia = 12.0, kPilotDia = 6.0;
    constexpr double kCx = 60.0, kCy = 40.0;

    process::ProcessPlan plan;
    {
        process::StepInvocation s;
        s.skill_id = "counterbore";
        s.params = {
            { "entry_face",    "top" },
            { "position_x_mm", kCx },
            { "position_y_mm", kCy },
            { "seat_dia_mm",   kSeatDia },
            { "seat_depth_mm", 4.0 },
            { "pilot_dia_mm",  kPilotDia },
            { "pilot_depth_mm", 12.0 },     // through the 12 mm plate
        };
        plan.append(s);
    }

    const TopoDS_Shape s1 = executeOnFreshStock(plan);
    int feat = -1;
    skill::Workpiece reim = stepRoundTrip(s1, feat);
    EXPECT_EQ(feat, 0) << "STEP must strip metadata (geometric recovery only)";

    // Find the counterbore candidate (measured seat/pilot diameters).
    bool found = false;
    for (const auto& c : re::analyze(reim)) {
        if (c.skill_id != "counterbore" || c.confidence < 0.7) continue;
        const auto& rp = c.recovered_params;
        EXPECT_NEAR(rp.value("seat_dia_mm",  0.0), kSeatDia,  0.10);
        EXPECT_NEAR(rp.value("pilot_dia_mm", 0.0), kPilotDia, 0.10);
        EXPECT_NEAR(rp.value("position_x_mm", 0.0), kCx, 0.20);
        EXPECT_NEAR(rp.value("position_y_mm", 0.0), kCy, 0.20);
        found = true;
        break;
    }
    EXPECT_TRUE(found) << "counterbore not recovered geometrically";
}

// ─── 5. The REAL product: watch case crown reframe, measured ──────────────
//
// A cylindrical watch case with a centre display pocket, a crown drill (3 o'
// clock) and a rear sensor drill.  The crown sub-assembly moves +6 mm in Y;
// the datum graph makes ONLY the crown drill follow while the sensor stays.
// Every claim is measured from a metadata-stripped STEP round-trip.
TEST(FrameReframeLoop, WatchCrownReframeMeasured)
{
    constexpr double kDia = 44.0, kThk = 10.0;
    constexpr double kCrownDia = 3.0, kSensorDia = 1.5;
    const XY kSensor = { -5.0, 15.0 };

    auto buildWatch = [&](double crownX, double crownY) {
        process::ProcessPlan p;
        {   process::StepInvocation s; s.skill_id = "mill_circular_pocket";
            s.params = { { "entry_face", "top" }, { "position_x_mm", 0.0 },
                         { "position_y_mm", 0.0 }, { "diameter_mm", 30.0 },
                         { "depth_mm", 6.0 } };
            p.append(s); }
        {   process::StepInvocation s; s.skill_id = "drill_hole";
            s.params = { { "entry_face", "top" }, { "position_x_mm", crownX },
                         { "position_y_mm", crownY }, { "diameter_mm", kCrownDia },
                         { "depth_mm", 2.5 }, { "through_hole", false } };
            p.append(s); }
        {   process::StepInvocation s; s.skill_id = "drill_hole";
            s.params = { { "entry_face", "top" }, { "position_x_mm", kSensor.x },
                         { "position_y_mm", kSensor.y }, { "diameter_mm", kSensorDia },
                         { "depth_mm", 2.0 }, { "through_hole", false } };
            p.append(s); }
        return p;
    };
    auto execWatch = [&](const process::ProcessPlan& p) {
        auto stock = skill::createCylindricalStock(kDia, kThk);
        auto res = process::Executor::execute(p, stock);
        EXPECT_TRUE(res.ok()) << (res.errors.empty() ? "" : res.errors.front());
        return res.workpiece->shape();
    };

    // Synth original watch -> STEP -> recover crown + sensor (measured).
    int feat = -1;
    skill::Workpiece reim = stepRoundTrip(execWatch(buildWatch(19.0, 0.0)), feat);
    ASSERT_EQ(feat, 0);
    const std::vector<RecHole> holes = recoverHoles(reim);
    const RecHole c0 = findNear(holes, 19.0,  0.0, 1.0);
    const RecHole s0 = findNear(holes, -5.0, 15.0, 1.0);
    ASSERT_GT(c0.dia, 0.0) << "crown drill not recovered";
    ASSERT_GT(s0.dia, 0.0) << "sensor drill not recovered";
    EXPECT_NEAR(c0.dia, kCrownDia,  0.08);   // measured crown diameter
    EXPECT_NEAR(s0.dia, kSensorDia, 0.08);   // measured sensor diameter

    // Crown sub-assembly moves +6 mm Y; sensor unchanged.
    process::ProcessPlan recoveredWatch = buildWatch(c0.x, c0.y);
    // The part sits at its real height in the case (z 0..10).  The drill steps
    // carry only x,y, so their z axis is ABSENT and correctly ignored by the
    // containment match — the part's z extent no longer has to be faked to 0.
    auto boxPart = [](const std::string& id, double cx, double cy) {
        parts::Part p; p.id = id;
        p.xMin = cx - 1.0; p.xMax = cx + 1.0;
        p.yMin = cy - 1.0; p.yMax = cy + 1.0;
        p.zMin = 0.0;      p.zMax = 10.0;
        return p;
    };
    parts::PartsLayout before, after;
    before.addPart(boxPart("crown",  19.0,  0.0));
    before.addPart(boxPart("sensor", -5.0, 15.0));
    after.addPart(boxPart("crown",  19.0,  6.0));   // +6 mm Y
    after.addPart(boxPart("sensor", -5.0, 15.0));   // stationary

    process::ProcessPlan adapted =
        parts::reframePlanForMoves(recoveredWatch, before, after, 1.0);

    int feat2 = -1;
    skill::Workpiece reim2 = stepRoundTrip(execWatch(adapted), feat2);
    ASSERT_EQ(feat2, 0);
    const std::vector<RecHole> holes2 = recoverHoles(reim2);

    EXPECT_GT(findNear(holes2, 19.0,  6.0, 1.0).dia, 0.0)
        << "crown drill must have followed the part to (19, 6)";
    EXPECT_GT(findNear(holes2, -5.0, 15.0, 1.0).dia, 0.0)
        << "sensor drill must remain at (-5, 15)";
    EXPECT_LT(findNear(holes2, 19.0,  0.0, 0.8).dia, 0.0)
        << "no crown drill should remain at the old (19, 0)";
}

// ─── 3. Volume is conserved across the reframe (same 4 holes, moved) ──────
TEST(FrameReframeLoop, VolumeConservedAcrossReframe)
{
    const TopoDS_Shape s1 = executeOnFreshStock(buildFramePlan(kHoles));

    std::vector<XY> shifted;
    for (const auto& h : kHoles) shifted.push_back({ h.x + 15.0, h.y });
    const TopoDS_Shape s2 = executeOnFreshStock(buildFramePlan(shifted));

    const double plate = kPlateX * kPlateY * kPlateZ;
    const double fourHoles = 4.0 * M_PI * (kHoleDia / 2.0) * (kHoleDia / 2.0) * kPlateZ;

    // Both frames removed the same 4-hole volume; result volumes match.
    EXPECT_NEAR(volumeOf(s1), volumeOf(s2), 1.0);
    EXPECT_NEAR(volumeOf(s1), plate - fourHoles, fourHoles * 0.05);
}
