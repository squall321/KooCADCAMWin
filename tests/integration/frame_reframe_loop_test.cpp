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

// ─── 2. Full reframe loop: parts move +15mm X -> holes follow ─────────────
TEST(FrameReframeLoop, ReframeShiftsHolesByPartDelta)
{
    constexpr double kDeltaX = 15.0;

    // Original frame -> STEP -> recover (geometric).
    const TopoDS_Shape s1 = executeOnFreshStock(buildFramePlan(kHoles));
    int feat = -1;
    skill::Workpiece reim = stepRoundTrip(s1, feat);
    ASSERT_EQ(feat, 0);
    const std::vector<RecHole> recovered = recoverHoles(reim);
    ASSERT_EQ(recovered.size(), kHoles.size());

    // ADAPT: the PCB part the holes anchor to shifted +15mm in X, so every
    // recovered hole's X is corrected by the part delta.  Build the adapted
    // plan straight from the recovered (measured) params.
    process::ProcessPlan adapted;
    for (const auto& h : recovered) {
        process::StepInvocation s;
        s.skill_id = "drill_hole";
        s.params = {
            { "entry_face",    "top" },
            { "position_x_mm", h.x + kDeltaX },
            { "position_y_mm", h.y },
            { "diameter_mm",   h.dia },
            { "depth_mm",      0.0 },
            { "through_hole",  true },
        };
        adapted.append(s);
    }

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
        << "after reframe, holes must sit at the part-shifted positions";

    // And NONE remain at an original X column (30 or 90).
    for (const auto& r : reframed) {
        EXPECT_GT(std::abs(r.x - 30.0), 1.0);
        EXPECT_GT(std::abs(r.x - 90.0), 1.0);
    }
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
