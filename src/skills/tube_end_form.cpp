// @lat: [[engine/skills#tube_end_form]]

#include "tube_end_form.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>

namespace koocadcam::skill::tube_end_form {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

bool isKnownFormType(const std::string& t)
{
    return t == "bell" || t == "beaded" || t == "expanded_slot";
}

bool isKnownEnd(const std::string& e)
{
    return e == "z_min" || e == "z_max";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!isKnownFormType(in.form_type)) {
        r.add("DFM-INPUT", "error",
              "tube_end_form form_type '" + in.form_type +
              "' not in {bell, beaded, expanded_slot}");
    }
    if (!isKnownEnd(in.end)) {
        r.add("DFM-TUBE-EF-END", "error",
              "tube_end_form end '" + in.end +
              "' not in {z_min, z_max}");
    }
    if (in.length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "tube_end_form length_mm (" + std::to_string(in.length_mm) +
              ") must be > 0");
    }
    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "tube_end_form diameter_mm (" + std::to_string(in.diameter_mm) +
              ") must be > 0");
    }

    if (!wp.shape().IsNull()) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        const double outerR = bb.outerRadiusXY();
        const double currentDia = 2.0 * outerR;

        if (in.length_mm > 0.0 && outerR > 0.0 && in.length_mm > 3.0 * outerR) {
            r.add("DFM-TUBE-EF-LEN", "warning",
                  "tube_end_form length_mm " + std::to_string(in.length_mm) +
                  " > 3 × outer radius (" + std::to_string(3.0 * outerR) +
                  ") — exceeds typical single-die stroke");
        }

        // For bell / beaded forms the diameter must increase past current OD.
        if ((in.form_type == "bell" || in.form_type == "beaded") &&
            in.diameter_mm > 0.0 && in.diameter_mm <= currentDia)
        {
            r.add("DFM-TUBE-EF-DIA", "error",
                  "tube_end_form '" + in.form_type + "' diameter_mm (" +
                  std::to_string(in.diameter_mm) +
                  ") must be > current OD (" + std::to_string(currentDia) + ")");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Metadata-only.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tube_end_form DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "form_type",   in.form_type },
        { "length_mm",   in.length_mm },
        { "diameter_mm", in.diameter_mm },
        { "end",         in.end },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "form_type",         in.form_type },
        { "length_mm",         in.length_mm },
        { "diameter_mm",       in.diameter_mm },
        { "end",               in.end },
        { "geometry_changed",  false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_form_die";
    tooling.tool_dia_mm       = in.diameter_mm;
    tooling.tool_length_mm    = in.length_mm;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 3.0 + in.length_mm * 0.2;
    tooling.extra = json{
        { "form_type",         in.form_type },
        { "end",               in.end },
        { "process_category",  "cold_forming" },
        { "geometric_no_op",   true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::tube_end_form applied: type={} len={} dia={} end={}",
                  in.form_type, in.length_mm, in.diameter_mm, in.end);

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

}  // namespace koocadcam::skill::tube_end_form
