// @lat: [[engine/skills#thread_mill]]
//
// ─── APPROXIMATION NOTE (slice 2) ────────────────────────────────────────
//
// A real thread-milled thread is identical in final geometry to a tapped
// thread — a helical groove (internal) or ridge (external).  The same
// MISSING PRIMITIVE blocks both skills:
//
//   MISSING PRIMITIVE:
//       prim::helicalSweep(
//           const gp_Ax1&   axis,
//           double          pitch_mm,
//           double          length_mm,
//           const TopoDS_Wire& profile_section);
//
//   Internally driven by Geom_Helix + BRepOffsetAPI_MakePipeShell.  Until
//   this primitive lands in src/engine/primitives/, thread_mill::apply()
//   is a metadata-only no-op identical in shape (but not in kind) to
//   tap_thread.  The signature.pattern.kind differs so a future
//   recognize() can refine the intent (helical-tool-path vs. tap).
//
//   When prim::helicalSweep arrives, swap the body for the real helical
//   pipe-cut and implement face-pattern based recognition.
//
// ─────────────────────────────────────────────────────────────────────────

#include "thread_mill.hpp"

#include "Workpiece.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::thread_mill {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.tool_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "thread_mill tool_dia_mm " + std::to_string(in.tool_dia_mm) +
              " < 0.8 mm (end-mill minimum)");
    }
    if (in.pitch_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "thread_mill pitch_mm must be > 0");
    }
    if (in.thread_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "thread_mill thread_depth_mm must be > 0");
    }
    if (in.pitch_mm > 0.0 && in.thread_depth_mm < in.pitch_mm * 1.5) {
        r.add("DFM-THREAD-ENGAGE", "error",
              "thread_depth_mm " + std::to_string(in.thread_depth_mm) +
              " < 1.5 × pitch (" + std::to_string(in.pitch_mm * 1.5) +
              ") — insufficient thread engagement");
    }
    (void)wp;
    return r;
}

// ── Synthesis (metadata-only approximation) ──────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "thread_mill DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto datumId = wp.resolve(in.existing_hole_datum);
    const int datumIdOut = datumId.value_or(-1);

    double datumDia = 0.0;
    if (datumId) {
        BRepAdaptor_Surface surf(wp.face(*datumId));
        if (surf.GetType() == GeomAbs_Cylinder) {
            datumDia = 2.0 * surf.Cylinder().Radius();
        }
    }

    json params = {
        { "datum_face_id",   datumIdOut },
        { "thread_size",     in.thread_size },
        { "pitch_mm",        in.pitch_mm },
        { "thread_depth_mm", in.thread_depth_mm },
        { "is_external",     in.is_external },
        { "tool_dia_mm",     in.tool_dia_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "thread_size",       in.thread_size },
        { "pitch_mm",          in.pitch_mm },
        { "thread_depth_mm",   in.thread_depth_mm },
        { "is_external",       in.is_external },
        { "datum_diameter_mm", datumDia },
        { "geometry",          "metadata_only" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "thread_mill";
    tooling.tool_dia_mm       = in.tool_dia_mm;
    tooling.tool_length_mm    = in.thread_depth_mm * 2.0 + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.02;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = std::max(2.0, in.thread_depth_mm / 3.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::thread_mill applied (metadata-only): {} pitch={} depth={} ext={}",
                  in.thread_size, in.pitch_mm, in.thread_depth_mm, in.is_external);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata-only) ──────────────────────────────────────────

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

}  // namespace koocadcam::skill::thread_mill
