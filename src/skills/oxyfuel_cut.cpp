// @lat: [[engine/skills#oxyfuel_cut]]

#include "oxyfuel_cut.hpp"

#include "Workpiece.hpp"
#include "_separation_common.hpp"
#include "engine/primitives/Cuts.hpp"

#include <Standard_Failure.hxx>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::oxyfuel_cut {

namespace sc = separation_common;
namespace pr = koocadcam::engine::prim;

namespace {

// Process limits for an oxy-acetylene torch.
//   - kerf 3.0 – 10.0 mm   (very wide — oxygen jet needs space)
//   - max thickness 300 mm
//   - finish: rough_oxidized (scale + dross)
//   - tool : "oxyfuel"
sc::ProcessLimits processLimits()
{
    return sc::ProcessLimits{
        /*skill_id*/         kSkillId,
        /*kerf_min_mm*/      3.0,
        /*kerf_max_mm*/      10.0,
        /*max_thickness_mm*/ 300.0,
        /*surface_finish*/   "rough_oxidized",
        /*tool_type*/        "oxyfuel",
        /*process_kerf_mm*/  6.5,   // bin centroid
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
        std::string msg = "oxyfuel_cut DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("oxyfuel_cut: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const sc::ProcessLimits lim = processLimits();
    const TopoDS_Shape cutter = sc::buildSeparationCutter(in, lim.process_kerf_mm,
                                                         zMin, zMax);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double thickness = zMax - zMin;
    FeatureSignature sig = sc::buildSignature(in, lim, *entryId, thickness, dfm);
    // Oxyfuel feed rate: ~10 mm/s in 25 mm steel, slower in thicker
    // plate (~2 mm/s at 200 mm).
    const double pathLen   = sc::pathLength(in);
    const double feedSpeed = std::max(1.0, 10.0 * (25.0 / std::max(1.0, thickness)));
    sig.tooling.est_cycle_time_s  = std::max(1.0, pathLen / feedSpeed);
    sig.tooling.tool_material     = "copper_nozzle";
    sig.tooling.cutting_speed_sfm = 0.0;
    sig.tooling.feed_per_tooth_mm = 0.0;
    sig.tooling.flute_count       = 0;
    sig.tooling.extra["preheat_gas"] = "acetylene";
    sig.tooling.extra["cut_gas"]     = "oxygen";
    sig.tooling.extra["machining_constraint"] =
        "Oxyfuel torch — carbon steel only (not stainless/Al/non-ferrous); "
        "very wide kerf with dross underside; through-cut only.";

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::oxyfuel_cut applied: kind={} thickness={} faces {}→{}",
                  sc::kindToString(in.kind), thickness,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return sc::recognizeForSkill(wp, kSkillId);
}

}  // namespace koocadcam::skill::oxyfuel_cut
