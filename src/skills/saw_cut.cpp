// @lat: [[engine/skills#saw_cut]]

#include "saw_cut.hpp"

#include "Workpiece.hpp"
#include "_separation_common.hpp"
#include "engine/primitives/Cuts.hpp"

#include <Standard_Failure.hxx>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::saw_cut {

namespace sc = separation_common;
namespace pr = koocadcam::engine::prim;

namespace {

// Process limits for a typical circular-saw blade.
//   - kerf 1.5 – 3.0 mm    (blade tooth set)
//   - max thickness varies wildly with blade diameter; we pick 100 mm as
//     a representative shop-floor table-saw / chop-saw upper bound.
//   - finish: rough (~Ra 6.3 μm)
//   - tool : "circular_saw"
//
// `process_kerf_mm` is the representative kerf the skill commits to; we
// pick 2.0 mm as the bin centroid.
sc::ProcessLimits processLimits()
{
    return sc::ProcessLimits{
        /*skill_id*/         kSkillId,
        /*kerf_min_mm*/      1.5,
        /*kerf_max_mm*/      3.0,
        /*max_thickness_mm*/ 100.0,
        /*surface_finish*/   "rough",
        /*tool_type*/        "circular_saw",
        /*process_kerf_mm*/  2.0,
    };
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    return sc::validateCommon(wp, in, processLimits());
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "saw_cut DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("saw_cut: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const sc::ProcessLimits lim = processLimits();
    const TopoDS_Shape cutter = sc::buildSeparationCutter(in, lim.process_kerf_mm,
                                                         zMin, zMax);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double thickness = zMax - zMin;
    FeatureSignature sig = sc::buildSignature(in, lim, *entryId, thickness, dfm);
    // Saw is FAST: ~50 mm/s linear traverse, scale by thickness.
    const double pathLen = sc::pathLength(in);
    sig.tooling.est_cycle_time_s = std::max(1.0, pathLen / 50.0 + thickness * 0.2);
    sig.tooling.tool_material    = "carbide_tipped_steel";
    sig.tooling.cutting_speed_sfm = 3000.0;
    sig.tooling.feed_per_tooth_mm = 0.10;
    sig.tooling.flute_count       = 60;     // typical blade tooth count
    sig.tooling.extra["machining_constraint"] =
        "Circular saw — wide kerf, fast, rough oxidised edge; through-cut only.";

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::saw_cut applied: kind={} thickness={} faces {}→{}",
                  sc::kindToString(in.kind), thickness,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return sc::recognizeForSkill(wp, kSkillId);
}

}  // namespace koocadcam::skill::saw_cut
