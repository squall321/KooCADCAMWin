// @lat: [[engine/skills#sandblast_op]]

#include "sandblast_op.hpp"

#include "Workpiece.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::sandblast_op {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pressure_psi <= 0.0) {
        r.add("DFM-INPUT", "error",
              "sandblast_op pressure_psi must be > 0");
    }
    if (in.pressure_psi > 100.0) {
        r.add("DFM-SANDBLAST-PRESSURE", "error",
              "sandblast_op pressure_psi " + std::to_string(in.pressure_psi) +
              " > 100 — risks cratering on aluminum / soft alloys");
    }
    if (in.grit_size.empty()) {
        r.add("DFM-INPUT", "warning",
              "sandblast_op grit_size empty — defaulting to 120_grit");
    }
    if (in.target_ra.empty()) {
        r.add("DFM-INPUT", "warning",
              "sandblast_op target_ra empty — defaulting to ra_3.2");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sandblast_op DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("sandblast_op: entry_face datum unresolved");

    const std::string grit = in.grit_size.empty()  ? "120_grit" : in.grit_size;
    const std::string ra   = in.target_ra.empty()  ? "ra_3.2"   : in.target_ra;

    // Surface area metadata — captured BEFORE we hand off the workpiece, so
    // a downstream tool can re-locate the blasted face by area.
    GProp_GProps props;
    BRepGProp::SurfaceProperties(wp.face(*entryId), props);
    const double faceArea = props.Mass();

    json params = {
        { "entry_face_id",   *entryId },
        { "grit_size",       grit },
        { "pressure_psi",    in.pressure_psi },
        { "target_ra",       ra },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "target_face_id",       *entryId },
        { "grit_size",            grit },
        { "target_ra",            ra },
        { "pressure_psi",         in.pressure_psi },
        { "target_face_area_mm2", faceArea },
        { "geometry_changed",     false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "sandblaster";
    tooling.tool_dia_mm       = 8.0;        // typical nozzle bore
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tungsten_carbide_nozzle";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;        // texturing-only
    // ~ 1 minute per 100 mm² at 60 psi.
    tooling.est_cycle_time_s  = std::max(5.0, faceArea / 100.0 * 60.0);
    tooling.extra = {
        { "grit_size",     grit },
        { "pressure_psi",  in.pressure_psi },
        { "target_ra",     ra },
        { "process",       "abrasive_blast" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // ── KEY DESIGN NOTE ──────────────────────────────────────────────────
    // Sandblasting does NOT change geometry — clone the shape verbatim.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::sandblast_op applied: face={} grit={} psi={} ra={}",
                  *entryId, grit, in.pressure_psi, ra);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Sandblasting leaves no macroscopic geometric trace.  We can only recover
// the operation by replaying the FeatureSignature history.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::sandblast_op
