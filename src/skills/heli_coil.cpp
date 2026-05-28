// @lat: [[engine/skills#heli_coil]]

#include "heli_coil.hpp"

#include "Workpiece.hpp"
#include "threaded_insert.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <string>

namespace koocadcam::skill::heli_coil {

using nlohmann::json;

// ── ISO 68-1 coarse-pitch defaults for supported sizes ───────────────────

namespace {

struct PitchSpec { const char* size; double pitch_mm; };
constexpr std::array<PitchSpec, 5> kCoarsePitchTable {{
    { "M3", 0.5 },
    { "M4", 0.7 },
    { "M5", 0.8 },
    { "M6", 1.0 },
    { "M8", 1.25 },
}};

}  // namespace

double defaultCoarsePitch(const std::string& thread_size)
{
    for (const auto& e : kCoarsePitchTable)
        if (thread_size == e.size) return e.pitch_mm;
    return 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    // Re-use threaded_insert's structural checks (size, length, material).
    threaded_insert::Input ti;
    ti.entry_face       = in.entry_face;
    ti.position_x_mm    = in.position_x_mm;
    ti.position_y_mm    = in.position_y_mm;
    ti.axis_dir         = in.axis_dir;
    ti.thread_size      = in.thread_size;
    ti.insert_length_mm = in.insert_length_mm;
    ti.insert_material  = in.insert_material;
    DFMReport inner = threaded_insert::validate(wp, ti);
    // Re-stamp the codes into heli_coil's namespace.
    for (const auto& f : inner.findings) {
        std::string code = f.code;
        if (code == "DFM-TI-SIZE") code = "DFM-HC-SIZE";
        else if (code == "DFM-TI-LEN") code = "DFM-HC-LEN";
        r.add(code, f.severity, f.message);
    }

    if (in.coil_pitch_mm <= 0.0) {
        r.add("DFM-HC-PITCH", "error",
              "heli_coil coil_pitch_mm must be > 0");
    }
    if (in.coil_count < 1 || in.coil_count > 3) {
        r.add("DFM-HC-COUNT", "error",
              "heli_coil coil_count " + std::to_string(in.coil_count) +
              " outside supported range [1, 3]");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "heli_coil DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Geometry: identical to threaded_insert.  Compose and then re-stamp.
    threaded_insert::Input ti;
    ti.entry_face       = in.entry_face;
    ti.position_x_mm    = in.position_x_mm;
    ti.position_y_mm    = in.position_y_mm;
    ti.axis_dir         = in.axis_dir;
    ti.thread_size      = in.thread_size;
    ti.insert_length_mm = in.insert_length_mm;
    ti.insert_material  = in.insert_material;

    SkillOutput inner = threaded_insert::apply(wp, ti);

    // Replace the wrapper feature signature with a heli_coil one.
    FeatureSignature sig = inner.signature;
    sig.skill_id = kSkillId;

    sig.params = {
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
        { "axis_dir",         { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "thread_size",      in.thread_size },
        { "insert_length_mm", in.insert_length_mm },
        { "insert_material",  in.insert_material },
        { "coil_pitch_mm",    in.coil_pitch_mm },
        { "coil_count",       in.coil_count },
        { "pilot_dia_mm",     inner.signature.params.value("pilot_dia_mm", 0.0) },
        { "drill_depth_mm",   inner.signature.params.value("drill_depth_mm", 0.0) },
    };
    sig.pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", true },
        { "thread_size",                in.thread_size },
        { "insert_length_mm",           in.insert_length_mm },
        { "coil_pitch_mm",              in.coil_pitch_mm },
        { "coil_count",                 in.coil_count },
        { "pilot_dia_mm",               inner.signature.params.value("pilot_dia_mm", 0.0) },
        { "axis_dir",                   { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };
    sig.tooling.tool_type = "drill;heli_coil_install";
    sig.tooling.extra["coil_pitch_mm"] = in.coil_pitch_mm;
    sig.tooling.extra["coil_count"]    = in.coil_count;
    sig.tooling.extra["note"] =
        "Heli-Coil wire insert: helical-thread geometry not modelled in "
        "slice-1; install op carried as metadata.";

    // The workpiece returned by threaded_insert already has the inner
    // feature; replace the most recent feature (threaded_insert) with our
    // re-stamped heli_coil signature so the history reflects the macro.
    auto wpNew = std::make_shared<Workpiece>(inner.workpiece->shape(),
                                              inner.workpiece->material());
    const auto& innerFeatures = inner.workpiece->features();
    // Copy everything EXCEPT the trailing threaded_insert feature, then
    // append our heli_coil signature.
    for (size_t i = 0; i + 1 < innerFeatures.size(); ++i) {
        wpNew->addFeature(innerFeatures[i]);
    }
    wpNew->addFeature(sig);

    spdlog::debug("skill::heli_coil applied: {} pitch={} coils={}",
                  in.thread_size, in.coil_pitch_mm, in.coil_count);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// History replay first.  Topology fallback re-uses threaded_insert's
// geometry detection but with a LOWER confidence — there is no way to
// distinguish a Heli-Coil install hole from a generic threaded insert
// install hole from B-rep alone.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "position_x_mm",    f.params.value("position_x_mm",    0.0) },
            { "position_y_mm",    f.params.value("position_y_mm",    0.0) },
            { "axis_dir",         f.params.value("axis_dir",         json::array({0.0,0.0,-1.0})) },
            { "thread_size",      f.params.value("thread_size",      std::string()) },
            { "insert_length_mm", f.params.value("insert_length_mm", 0.0) },
            { "insert_material",  f.params.value("insert_material",  std::string()) },
            { "coil_pitch_mm",    f.params.value("coil_pitch_mm",    0.0) },
            { "coil_count",       f.params.value("coil_count",       1) },
            { "pilot_dia_mm",     f.params.value("pilot_dia_mm",     0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.95, json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Topology fallback — borrow the threaded_insert search and re-tag.
    auto tiCands = threaded_insert::recognize(wp);
    for (const auto& c : tiCands) {
        const std::string size = c.recovered_params.value("thread_size", std::string());
        if (size.empty()) continue;
        json rec = {
            { "position_x_mm",    c.recovered_params.value("position_x_mm",    0.0) },
            { "position_y_mm",    c.recovered_params.value("position_y_mm",    0.0) },
            { "axis_dir",         c.recovered_params.value("axis_dir",         json::array({0.0,0.0,-1.0})) },
            { "thread_size",      size },
            { "insert_length_mm", c.recovered_params.value("insert_length_mm", 0.0) },
            { "insert_material",  c.recovered_params.value("insert_material",  std::string("unknown")) },
            { "coil_pitch_mm",    defaultCoarsePitch(size) },
            { "coil_count",       1 },
            { "pilot_dia_mm",     c.recovered_params.value("pilot_dia_mm",     0.0) },
        };
        // Confidence noticeably below threaded_insert (cannot distinguish).
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.35, c.matched_geometry
        });
    }
    return out;
}

}  // namespace koocadcam::skill::heli_coil
