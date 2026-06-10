// @lat: [[engine/skills#plasma_cut]]

#include "plasma_cut.hpp"

#include "Workpiece.hpp"
#include "_separation_common.hpp"
#include "engine/primitives/Cuts.hpp"

#include <Standard_Failure.hxx>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::plasma_cut {

namespace sc = separation_common;
namespace pr = koocadcam::engine::prim;

namespace {

// Process limits for a plasma arc cutter.
//   - kerf 1.0 – 6.0 mm    (torch / current dependent)
//   - max thickness 50 mm
//   - finish: rough_oxidized (heat-affected, scaled edge)
//   - tool : "plasma"
sc::ProcessLimits processLimits()
{
    return sc::ProcessLimits{
        /*skill_id*/         kSkillId,
        /*kerf_min_mm*/      1.0,
        /*kerf_max_mm*/      6.0,
        /*max_thickness_mm*/ 50.0,
        /*surface_finish*/   "rough_oxidized",
        /*tool_type*/        "plasma",
        /*process_kerf_mm*/  3.5,    // bin centroid (mid-range torch)
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
        std::string msg = "plasma_cut DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("plasma_cut: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const sc::ProcessLimits lim = processLimits();
    const TopoDS_Shape cutter = sc::buildSeparationCutter(in, lim.process_kerf_mm,
                                                         zMin, zMax);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double thickness = zMax - zMin;
    FeatureSignature sig = sc::buildSignature(in, lim, *entryId, thickness, dfm);
    // Plasma is FAST: ~40 mm/s in 10 mm mild steel; scales linearly with
    // thickness (slower in thicker stock).
    const double pathLen   = sc::pathLength(in);
    const double feedSpeed = std::max(5.0, 40.0 * (10.0 / std::max(1.0, thickness)));
    sig.tooling.est_cycle_time_s  = std::max(1.0, pathLen / feedSpeed);
    sig.tooling.tool_material     = "n/a";  // ionised gas + electrode
    sig.tooling.cutting_speed_sfm = 0.0;
    sig.tooling.feed_per_tooth_mm = 0.0;
    sig.tooling.flute_count       = 0;
    sig.tooling.extra["plasma_gas"] = "air | N2 | O2 (material dependent)";
    sig.tooling.extra["machining_constraint"] =
        "Plasma torch — conductive materials only (steel, Al, SS); "
        "rough oxidised edge with scale; through-cut only.";

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::plasma_cut applied: kind={} thickness={} faces {}→{}",
                  sc::kindToString(in.kind), thickness,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return sc::recognizeForSkill(wp, kSkillId);
}

}  // namespace koocadcam::skill::plasma_cut
