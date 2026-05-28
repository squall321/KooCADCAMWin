// @lat: [[engine/skills#laser_cut]]

#include "laser_cut.hpp"

#include "Workpiece.hpp"
#include "_separation_common.hpp"
#include "engine/primitives/Cuts.hpp"

#include <Standard_Failure.hxx>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::laser_cut {

namespace sc = separation_common;
namespace pr = koocadcam::engine::prim;

namespace {

// Process limits for a fibre / CO2 laser cutter.
//   - kerf 0.05 – 0.5 mm  (spot-size limited)
//   - max thickness 25 mm steel (less for stainless / aluminium / brass)
//   - finish: fine (small HAZ but sharp edge)
//   - tool : "laser"
sc::ProcessLimits processLimits()
{
    return sc::ProcessLimits{
        /*skill_id*/         kSkillId,
        /*kerf_min_mm*/      0.05,
        /*kerf_max_mm*/      0.5,
        /*max_thickness_mm*/ 25.0,
        /*surface_finish*/   "fine",
        /*tool_type*/        "laser",
        /*process_kerf_mm*/  0.2,   // bin centroid
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
        std::string msg = "laser_cut DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("laser_cut: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const sc::ProcessLimits lim = processLimits();
    const TopoDS_Shape cutter = sc::buildSeparationCutter(in, lim.process_kerf_mm,
                                                         zMin, zMax);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double thickness = zMax - zMin;
    FeatureSignature sig = sc::buildSignature(in, lim, *entryId, thickness, dfm);
    // Laser is FAST in thin stock (50-100 mm/s in 3 mm steel) but rapidly
    // slows in thick stock (~5 mm/s in 25 mm steel).
    const double pathLen   = sc::pathLength(in);
    const double feedSpeed = std::max(2.0, 100.0 / std::max(1.0, thickness));
    sig.tooling.est_cycle_time_s  = std::max(0.5, pathLen / feedSpeed);
    sig.tooling.tool_material     = "n/a";   // photons + assist gas
    sig.tooling.cutting_speed_sfm = 0.0;
    sig.tooling.feed_per_tooth_mm = 0.0;
    sig.tooling.flute_count       = 0;
    sig.tooling.extra["assist_gas"] = "N2 (stainless) / O2 (mild steel)";
    sig.tooling.extra["machining_constraint"] =
        "Fibre / CO2 laser — fine kerf, small HAZ, fast in thin stock; "
        "through-cut only.";

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::laser_cut applied: kind={} thickness={} faces {}→{}",
                  sc::kindToString(in.kind), thickness,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return sc::recognizeForSkill(wp, kSkillId);
}

}  // namespace koocadcam::skill::laser_cut
