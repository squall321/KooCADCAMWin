// @lat: [[engine/skills#lens_grind]]

#include "lens_grind.hpp"

#include "Workpiece.hpp"
#include "_coating_common.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::lens_grind {

namespace cc = koocadcam::skill::coating_common;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.aperture_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "lens_grind aperture_mm must be > 0 (got " +
              std::to_string(in.aperture_mm) + ")");
    }

    // Plano (radius == 0) is acceptable; otherwise |R| must accommodate
    // the requested aperture as a spherical cap: |R| ≥ aperture/2.
    if (in.radius_curvature_mm != 0.0 && in.aperture_mm > 0.0) {
        const double half_ap = in.aperture_mm * 0.5;
        if (std::fabs(in.radius_curvature_mm) < half_ap) {
            r.add("DFM-LENS-GEOM", "error",
                  "lens_grind |radius_curvature_mm|=" +
                  std::to_string(std::fabs(in.radius_curvature_mm)) +
                  " < aperture/2=" + std::to_string(half_ap) +
                  " — spherical cap impossible for requested aperture");
        }
    }

    if (in.surface_quality.empty()) {
        r.add("DFM-LENS-QUAL", "warning",
              "lens_grind surface_quality empty — defaulting to commercial grade");
    } else if (in.surface_quality.find('/') == std::string::npos) {
        r.add("DFM-LENS-QUAL", "info",
              "lens_grind surface_quality '" + in.surface_quality +
              "' does not look like ISO 10110-7 scratch/dig format (expected 'N/CxA')");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lens_grind DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("lens_grind: entry_face datum unresolved");

    const std::string surface_quality =
        in.surface_quality.empty() ? "commercial" : in.surface_quality;

    const std::string curvature_kind =
        in.radius_curvature_mm == 0.0  ? "plano"
      : in.radius_curvature_mm  > 0.0  ? "convex"
                                       : "concave";

    json params = {
        { "entry_face_id",       *entryId },
        { "radius_curvature_mm", in.radius_curvature_mm },
        { "aperture_mm",         in.aperture_mm },
        { "surface_quality",     surface_quality },
    };
    json pattern = {
        { "kind",                "lens_grind" },
        { "target_face_id",      *entryId },
        { "radius_curvature_mm", in.radius_curvature_mm },
        { "aperture_mm",         in.aperture_mm },
        { "surface_quality",     surface_quality },
        { "curvature_kind",      curvature_kind },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "diamond_pellet_lap";
    tooling.tool_dia_mm       = in.aperture_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "bonded_diamond";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // recorded as metadata only
    // Empirical: grind cycle ~ aperture_mm seconds, floored at 60 s.
    tooling.est_cycle_time_s  = std::max(60.0, in.aperture_mm * 1.0);
    tooling.extra = {
        { "process",             "lens_grind" },
        { "curvature_kind",      curvature_kind },
        { "radius_curvature_mm", in.radius_curvature_mm },
        { "aperture_mm",         in.aperture_mm },
        { "surface_quality",     surface_quality },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto stamped = cc::stampNoOpCoating(wp, *entryId, sig);

    spdlog::debug("skill::lens_grind applied: face={} R={}mm aperture={}mm",
                  *entryId, in.radius_curvature_mm, in.aperture_mm);

    return SkillOutput{ stamped.workpiece, stamped.signature };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return cc::recognizeFromMetadata(wp, kSkillId);
}

}  // namespace koocadcam::skill::lens_grind
