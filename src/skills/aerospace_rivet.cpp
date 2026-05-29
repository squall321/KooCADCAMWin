// @lat: [[engine/skills#aerospace_rivet]]

#include "aerospace_rivet.hpp"

#include "Workpiece.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <string>

namespace koocadcam::skill::aerospace_rivet {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.rivet_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "aerospace_rivet rivet_dia_mm must be > 0");
    } else {
        if (in.rivet_dia_mm < 2.4) {
            r.add("DFM-RIVET-DIA", "error",
                  "aerospace_rivet rivet_dia_mm " +
                  std::to_string(in.rivet_dia_mm) +
                  " < 2.4 mm — below smallest aerospace structural rivet");
        }
        if (in.rivet_dia_mm > 8.0) {
            r.add("DFM-RIVET-DIA", "error",
                  "aerospace_rivet rivet_dia_mm " +
                  std::to_string(in.rivet_dia_mm) +
                  " > 8.0 mm — use bolted joint or Hi-Lok for larger diameters");
        }
    }

    if (in.length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "aerospace_rivet length_mm must be > 0");
    } else if (in.rivet_dia_mm > 0.0) {
        if (in.length_mm < 1.5 * in.rivet_dia_mm) {
            r.add("DFM-RIVET-LEN", "error",
                  "aerospace_rivet length_mm " + std::to_string(in.length_mm) +
                  " < 1.5 × rivet_dia (" + std::to_string(1.5 * in.rivet_dia_mm) +
                  ") — insufficient grip for tail upset");
        }
        if (in.length_mm > 10.0 * in.rivet_dia_mm) {
            r.add("DFM-RIVET-LEN", "error",
                  "aerospace_rivet length_mm " + std::to_string(in.length_mm) +
                  " > 10 × rivet_dia (" + std::to_string(10.0 * in.rivet_dia_mm) +
                  ") — grip stack too thick, switch to bolt");
        }
    }

    if (in.alloy.empty()) {
        r.add("DFM-INPUT", "warning",
              "aerospace_rivet alloy empty — defaulting to AA2117");
    }

    if (in.style != "blind" && in.style != "solid") {
        r.add("DFM-RIVET-STYLE", "info",
              "aerospace_rivet style '" + in.style +
              "' not in {blind, solid} — treated as solid for planning");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "aerospace_rivet DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("aerospace_rivet: entry_face datum unresolved");

    const std::string alloy = in.alloy.empty() ? "AA2117" : in.alloy;
    const std::string style = (in.style == "blind" || in.style == "solid")
                                  ? in.style : "solid";

    GProp_GProps props;
    BRepGProp::SurfaceProperties(wp.face(*entryId), props);
    const double faceArea = props.Mass();

    json params = {
        { "entry_face_id", *entryId },
        { "position_x_mm", in.position_x_mm },
        { "position_y_mm", in.position_y_mm },
        { "rivet_dia_mm",  in.rivet_dia_mm },
        { "length_mm",     in.length_mm },
        { "alloy",         alloy },
        { "style",         style },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "target_face_id",       *entryId },
        { "rivet_dia_mm",         in.rivet_dia_mm },
        { "length_mm",            in.length_mm },
        { "alloy",                alloy },
        { "style",                style },
        { "head_style",           "flush_100deg" },
        { "target_face_area_mm2", faceArea },
        { "grip_ratio",
          in.rivet_dia_mm > 0.0 ? (in.length_mm / in.rivet_dia_mm) : 0.0 },
        { "geometry_changed",     false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = (style == "blind") ? "blind_rivet_gun"
                                                   : "pneumatic_bucking_bar";
    tooling.tool_dia_mm       = in.rivet_dia_mm;
    tooling.tool_length_mm    = in.length_mm;
    tooling.tool_material     = (alloy == "monel") ? "monel" : "aluminum_alloy";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // installation, no removal
    // Hand-set blind rivet ~ 2 s; bucked solid rivet ~ 4 s.
    tooling.est_cycle_time_s  = (style == "blind") ? 2.0 : 4.0;
    tooling.extra = {
        { "process",        "structural_rivet_install" },
        { "alloy",          alloy },
        { "style",          style },
        { "head_style",     "flush_100deg" },
        { "rivet_dia_mm",   in.rivet_dia_mm },
        { "length_mm",      in.length_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // No geometric change — clone shape + material verbatim.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::aerospace_rivet applied: face={} dia={} len={} alloy={} style={}",
                  *entryId, in.rivet_dia_mm, in.length_mm, alloy, style);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Installed rivet body is a sub-OCCT entity — recognition is metadata-only.

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

}  // namespace koocadcam::skill::aerospace_rivet
