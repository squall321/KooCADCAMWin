// @lat: [[engine/skills#tube_bend]]

#include "tube_bend.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

namespace koocadcam::skill::tube_bend {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bend_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "tube_bend bend_radius_mm (" + std::to_string(in.bend_radius_mm) +
              ") must be > 0");
    }
    if (in.bend_angle_deg <= 0.0 || in.bend_angle_deg > 180.0) {
        r.add("DFM-INPUT", "error",
              "tube_bend bend_angle_deg (" + std::to_string(in.bend_angle_deg) +
              ") must be in (0, 180]");
    }

    if (!wp.shape().IsNull()) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        const double outerR = bb.outerRadiusXY();

        // Centerline-bend-radius rule: CLR ≥ 1.5 × OD/2 = 1.5 × outerR.
        if (in.bend_radius_mm > 0.0 && outerR > 0.0 &&
            in.bend_radius_mm < 1.5 * outerR)
        {
            r.add("DFM-TUBE-BEND", "error",
                  "tube_bend bend_radius_mm " +
                  std::to_string(in.bend_radius_mm) +
                  " < 1.5 × outer radius (" + std::to_string(1.5 * outerR) +
                  ") — inside-wall buckling risk");
        }

        if (in.axial_position_mm < bb.zMin - 1e-3 ||
            in.axial_position_mm > bb.zMax + 1e-3)
        {
            r.add("DFM-TUBE-BEND", "error",
                  "tube_bend axial_position_mm " +
                  std::to_string(in.axial_position_mm) +
                  " outside stock Z [" + std::to_string(bb.zMin) + "," +
                  std::to_string(bb.zMax) + "]");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Metadata-only: shape is preserved, signature carries the bend parameters.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tube_bend DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "bend_angle_deg",    in.bend_angle_deg },
        { "bend_radius_mm",    in.bend_radius_mm },
        { "axial_position_mm", in.axial_position_mm },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "bend_angle_deg",     in.bend_angle_deg },
        { "bend_radius_mm",     in.bend_radius_mm },
        { "axial_position_mm",  in.axial_position_mm },
        { "geometry_changed",   false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "draw_bender";
    tooling.tool_dia_mm       = 2.0 * in.bend_radius_mm;
    tooling.tool_length_mm    = in.bend_radius_mm * in.bend_angle_deg * M_PI / 180.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 5.0 + in.bend_angle_deg * 0.05;
    {
        double outerR = 0.0;
        if (!wp.shape().IsNull()) {
            outerR = pr::optimalBbox(wp.shape()).outerRadiusXY();
        }
        const double clrRatio = (outerR > 0.0)
            ? in.bend_radius_mm / outerR : 0.0;
        tooling.extra = json{
            { "centerline_radius_ratio", clrRatio },
            { "process_category",        "cold_forming" },
            { "geometric_no_op",         true },
        };
    }

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::tube_bend applied: angle={}° R={} at z={} (metadata only)",
                  in.bend_angle_deg, in.bend_radius_mm, in.axial_position_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata replay only — geometry is unchanged.

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

}  // namespace koocadcam::skill::tube_bend
