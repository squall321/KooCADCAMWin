// @lat: [[engine/skills#reference_axis_two_pts]]
//
// Metadata-only reference axis: workpiece shape unchanged; axis (origin =
// point_a, dir = unit(B - A), length = |B - A|) captured in FeatureSignature.

#include "reference_axis_two_pts.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::reference_axis_two_pts {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.name.empty()) {
        r.add("DFM-INPUT", "error",
              "reference_axis_two_pts: name must be non-empty");
    }
    const double dx = in.point_b[0] - in.point_a[0];
    const double dy = in.point_b[1] - in.point_a[1];
    const double dz = in.point_b[2] - in.point_a[2];
    const double d  = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (d < 1e-3) {
        r.add("DFM-REF-COINCIDENT", "error",
              "reference_axis_two_pts: |B - A| = " + std::to_string(d) +
              " < 1e-3 — points are coincident, axis is undefined");
    }
    return r;
}

// ── Synthesis (metadata only) ────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "reference_axis_two_pts DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double dx = in.point_b[0] - in.point_a[0];
    const double dy = in.point_b[1] - in.point_a[1];
    const double dz = in.point_b[2] - in.point_a[2];
    const double L  = std::sqrt(dx * dx + dy * dy + dz * dz);
    // L > 1e-3 guaranteed by validate.
    const double ux = dx / L;
    const double uy = dy / L;
    const double uz = dz / L;

    json params = {
        { "point_a", in.point_a },
        { "point_b", in.point_b },
        { "name",    in.name },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_reference",      true },
        { "datum_class",       "axis" },
        { "named_id",          in.name },
        { "point_a_xyz",       in.point_a },
        { "point_b_xyz",       in.point_b },
        { "axis_origin_xyz",   in.point_a },
        { "axis_dir_xyz",      { ux, uy, uz } },
        { "axis_length",       L },
        { "geometry_changed",  false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "reference_datum";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "datum_class",     "axis" },
        { "geometric_no_op", true },
        { "named_id",        in.name },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::reference_axis_two_pts: name={} L={}",
                  in.name, L);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "feature_history" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::reference_axis_two_pts
