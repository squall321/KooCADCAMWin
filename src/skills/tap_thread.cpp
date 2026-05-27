// @lat: [[engine/skills#tap_thread]]
//
// ─── APPROXIMATION NOTE (slice 2) ────────────────────────────────────────
//
// A true tap_thread leaves a helical groove on the bore wall.  Modelling
// that helix as B-rep geometry requires:
//
//   MISSING PRIMITIVE:
//       prim::helicalSweep(
//           const gp_Ax1&   axis,           // bore axis
//           double          pitch_mm,
//           double          length_mm,
//           const TopoDS_Wire& profile_section);   // V-thread cross-section
//
//   which would internally build a Geom_Helix-driven wire and feed it to
//   BRepOffsetAPI_MakePipe (or BRepOffsetAPI_MakePipeShell).  This primitive
//   is NOT yet wrapped in src/engine/primitives/.  Until it lands (slice 4),
//   tap_thread::apply() is a metadata-only no-op:
//
//     • Workpiece geometry is returned UNCHANGED.
//     • A FeatureSignature with pattern.geometry == "metadata_only" is
//       recorded so process plans can address tap intent.
//     • recognize() can only return previously registered signatures —
//       it cannot rediscover taps after STEP round-trip.
//
//   When prim::helicalSweep arrives, swap the no-op body in apply() for the
//   real pipe-along-helix cut and add proper face-pattern recognition.
//
// ─────────────────────────────────────────────────────────────────────────

#include "tap_thread.hpp"

#include "Workpiece.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::tap_thread {

using nlohmann::json;

// ── Tap-drill chart (ISO metric coarse, from DFM-015 references) ─────────

double tapDrillDiameter(const std::string& thread_size)
{
    // Standard ISO metric coarse tap-drill chart (mm).
    if (thread_size == "M1.6") return 1.25;
    if (thread_size == "M2")   return 1.60;
    if (thread_size == "M2.5") return 2.05;
    if (thread_size == "M3")   return 2.50;
    if (thread_size == "M4")   return 3.30;
    if (thread_size == "M5")   return 4.20;
    if (thread_size == "M6")   return 5.00;
    if (thread_size == "M8")   return 6.80;
    if (thread_size == "M10")  return 8.50;
    return 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pitch_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "tap_thread pitch_mm must be > 0");
    }
    if (in.thread_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "tap_thread thread_depth_mm must be > 0");
    }
    if (in.pitch_mm > 0.0 && in.thread_depth_mm < in.pitch_mm * 1.5) {
        r.add("DFM-TAP-ENGAGE", "error",
              "thread_depth_mm " + std::to_string(in.thread_depth_mm) +
              " < 1.5 × pitch (" + std::to_string(in.pitch_mm * 1.5) +
              ") — insufficient thread engagement");
    }

    const double expected = tapDrillDiameter(in.thread_size);
    if (expected <= 0.0) {
        r.add("DFM-TAP-SIZE", "warning",
              "unknown thread size '" + in.thread_size + "' — tap-drill chart miss");
    } else {
        auto pilotId = wp.resolve(in.existing_hole_datum);
        if (pilotId) {
            BRepAdaptor_Surface surf(wp.face(*pilotId));
            if (surf.GetType() == GeomAbs_Cylinder) {
                const double pilotDia = 2.0 * surf.Cylinder().Radius();
                // DFM-015: pilot must be within ±0.1 mm of the chart value.
                if (std::abs(pilotDia - expected) > 0.1) {
                    r.add("DFM-015", "warning",
                          "pilot diameter " + std::to_string(pilotDia) +
                          " mm does not match tap-drill chart for " +
                          in.thread_size + " (expected " +
                          std::to_string(expected) + " mm)");
                }
            } else {
                r.add("DFM-TAP-DATUM", "warning",
                      "existing_hole_datum did not resolve to a cylindrical face");
            }
        } else {
            r.add("DFM-TAP-DATUM", "warning",
                  "existing_hole_datum unresolved — cannot verify pilot diameter");
        }
    }

    return r;
}

// ── Synthesis (metadata-only approximation) ──────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    // Only block on actual errors; warnings (size mismatch) pass through.
    if (!dfm.passed) {
        std::string msg = "tap_thread DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto pilotId = wp.resolve(in.existing_hole_datum);
    // Best-effort: resolution failure becomes a -1 pilot id in metadata,
    // but the synthesis still succeeds (no geometry change).
    const int pilotIdOut = pilotId.value_or(-1);

    double pilotDia = 0.0;
    if (pilotId) {
        BRepAdaptor_Surface surf(wp.face(*pilotId));
        if (surf.GetType() == GeomAbs_Cylinder) {
            pilotDia = 2.0 * surf.Cylinder().Radius();
        }
    }

    // ─── Build signature (metadata only — no geometric change) ───────────
    json params = {
        { "pilot_face_id",   pilotIdOut },
        { "thread_size",     in.thread_size },
        { "pitch_mm",        in.pitch_mm },
        { "thread_depth_mm", in.thread_depth_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "thread_size",      in.thread_size },
        { "pitch_mm",         in.pitch_mm },
        { "thread_depth_mm",  in.thread_depth_mm },
        { "pilot_diameter_mm", pilotDia },
        { "geometry",         "metadata_only" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "tap";
    tooling.tool_dia_mm       = pilotDia;
    tooling.tool_length_mm    = in.thread_depth_mm * 2.0 + 10.0;
    tooling.tool_material     = "hss";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 30.0;   // taps run slow
    tooling.feed_per_tooth_mm = in.pitch_mm / 4.0;
    tooling.stock_removed_mm3 = 0.0;    // helical groove volume tiny vs. pilot
    tooling.est_cycle_time_s  = std::max(2.0, in.thread_depth_mm / 5.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // ─── Geometry no-op: clone workpiece, just register feature ──────────
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    // Preserve existing feature history so prior skills remain queryable.
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::tap_thread applied (metadata-only): {} pitch={} depth={}",
                  in.thread_size, in.pitch_mm, in.thread_depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata-only) ──────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    // Until prim::helicalSweep arrives, the only way to find a tap is to
    // replay registered FeatureSignatures.  Post-STEP-round-trip workpieces
    // lose this metadata and yield an empty result — by design.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;  // exact match — we synthesized it
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::tap_thread
