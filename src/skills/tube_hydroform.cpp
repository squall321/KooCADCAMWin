// @lat: [[engine/skills#tube_hydroform]]

#include "tube_hydroform.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace koocadcam::skill::tube_hydroform {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.target_profile_polyline.size() < 2) {
        r.add("DFM-INPUT", "error",
              "tube_hydroform target_profile_polyline must have ≥ 2 vertices");
    }

    // Monotone Z + all R > 0 + accumulate min/max R.
    double rMin = std::numeric_limits<double>::infinity();
    double rMax = 0.0;
    double zMin = std::numeric_limits<double>::infinity();
    double zMax = -std::numeric_limits<double>::infinity();
    bool monotone = true;
    for (size_t i = 0; i < in.target_profile_polyline.size(); ++i) {
        const auto& v = in.target_profile_polyline[i];
        const double z = v[0];
        const double rad = v[1];
        if (rad <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "tube_hydroform polyline vertex " + std::to_string(i) +
                  " has non-positive outer_R (" + std::to_string(rad) + ")");
        }
        if (rad < rMin) rMin = rad;
        if (rad > rMax) rMax = rad;
        if (z < zMin) zMin = z;
        if (z > zMax) zMax = z;
        if (i > 0) {
            const double zPrev = in.target_profile_polyline[i - 1][0];
            if (z <= zPrev) monotone = false;
        }
    }
    if (in.target_profile_polyline.size() >= 2 && !monotone) {
        r.add("DFM-INPUT", "error",
              "tube_hydroform polyline must be strictly Z-monotone");
    }

    // Expansion ratio.
    if (rMin > 0.0 && rMax > 0.0 && std::isfinite(rMin)) {
        const double ratio = rMax / rMin;
        if (ratio > 1.6) {
            r.add("DFM-TUBE-HF-RATIO", "warning",
                  "tube_hydroform max/min R ratio " + std::to_string(ratio) +
                  " > 1.6 — single-stage hydroform expansion limit exceeded");
        }
    }

    // Pressure.
    if (in.pressure_mpa < 50.0 || in.pressure_mpa > 600.0) {
        r.add("DFM-TUBE-HF-PRESS", "error",
              "tube_hydroform pressure_mpa " + std::to_string(in.pressure_mpa) +
              " outside [50, 600] MPa");
    }

    // Z range overlap.
    if (!wp.shape().IsNull() && std::isfinite(zMin) && std::isfinite(zMax)) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        if (zMin > bb.zMax + 1e-3 || zMax < bb.zMin - 1e-3) {
            r.add("DFM-TUBE-HF-Z", "error",
                  "tube_hydroform polyline Z range [" + std::to_string(zMin) +
                  "," + std::to_string(zMax) + "] does not overlap stock Z [" +
                  std::to_string(bb.zMin) + "," + std::to_string(bb.zMax) + "]");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Metadata-only.  Geometry unchanged.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tube_hydroform DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Serialize the polyline.
    json polylineJ = json::array();
    for (const auto& v : in.target_profile_polyline) {
        polylineJ.push_back({ v[0], v[1] });
    }

    json params = {
        { "target_profile_polyline", polylineJ },
        { "pressure_mpa",            in.pressure_mpa },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "target_profile_polyline", polylineJ },
        { "pressure_mpa",            in.pressure_mpa },
        { "vertex_count",            static_cast<int>(in.target_profile_polyline.size()) },
        { "geometry_changed",        false },
    };

    // Compute polyline geometric summary (rMin/rMax/length).
    double rMin = std::numeric_limits<double>::infinity();
    double rMax = 0.0;
    for (const auto& v : in.target_profile_polyline) {
        rMin = std::min(rMin, v[1]);
        rMax = std::max(rMax, v[1]);
    }
    if (!std::isfinite(rMin)) rMin = 0.0;
    const double expansionRatio = (rMin > 0.0) ? rMax / rMin : 0.0;

    ToolingMeta tooling;
    tooling.tool_type         = "hydroform_die";
    tooling.tool_dia_mm       = 2.0 * rMax;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 30.0 + 0.5 * in.pressure_mpa;
    tooling.extra = json{
        { "expansion_ratio",  expansionRatio },
        { "process_category", "cold_forming" },
        { "geometric_no_op",  true },
        { "pressure_mpa",     in.pressure_mpa },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::tube_hydroform applied: {} vertices, pressure={} MPa",
                  static_cast<int>(in.target_profile_polyline.size()),
                  in.pressure_mpa);

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
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::tube_hydroform
