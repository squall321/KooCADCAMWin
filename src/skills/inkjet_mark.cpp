// @lat: [[engine/skills#inkjet_mark]]
//
// Inkjet marking is a GEOMETRIC NO-OP — we clone the workpiece shape
// verbatim and stamp a FeatureSignature carrying text, colour, font, and
// target face metadata.  Pattern follows the sandblast_op precedent for
// metadata-only operations: capture the target face area so downstream
// consumers can re-locate the printed face by surface area after a STEP
// round-trip.

#include "inkjet_mark.hpp"

#include "Workpiece.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::inkjet_mark {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.font_size_mm < 0.3) {
        r.add("DFM-INKJET-FONT", "error",
              "inkjet_mark font_size_mm " + std::to_string(in.font_size_mm) +
              " < 0.3 mm (inkjet droplet resolution limit)");
    }
    if (in.text.empty()) {
        r.add("DFM-INPUT", "error", "inkjet_mark text must not be empty");
    }
    if (in.ink_color.empty()) {
        r.add("DFM-INPUT", "warning",
              "inkjet_mark ink_color empty — defaulting to 'black'");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "inkjet_mark DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("inkjet_mark: entry_face datum unresolved");

    const std::string color = in.ink_color.empty() ? "black" : in.ink_color;

    // Capture target face area so re-import can relocate.
    GProp_GProps props;
    BRepGProp::SurfaceProperties(wp.face(*entryId), props);
    const double faceArea = props.Mass();

    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "text",            in.text },
        { "font_size_mm",    in.font_size_mm },
        { "ink_color",       color },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "text",                 in.text },
        { "char_count",           in.text.size() },
        { "font_size_mm",         in.font_size_mm },
        { "ink_color",            color },
        { "target_face_id",       *entryId },
        { "target_face_area_mm2", faceArea },
        { "geometry_changed",     false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "inkjet_printhead";
    tooling.tool_dia_mm       = std::max(0.05, in.font_size_mm * 0.1);
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "piezo_nozzle";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // pure surface ink
    // ~ 0.5 s per character — inkjet printheads are very fast.
    tooling.est_cycle_time_s  = std::max(0.5,
                                static_cast<double>(in.text.size()) * 0.3);
    tooling.extra = {
        { "process",    "inkjet_print" },
        { "ink_color",  color },
        { "geometric_noop", true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // ── KEY DESIGN NOTE ──────────────────────────────────────────────────
    // Inkjet marking does NOT change geometry — clone the shape verbatim.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::inkjet_mark applied: '{}' color={} font={}",
                  in.text, color, in.font_size_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Ink leaves no macroscopic geometric trace.  We can only recover the
// operation by replaying the FeatureSignature history.

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

}  // namespace koocadcam::skill::inkjet_mark
