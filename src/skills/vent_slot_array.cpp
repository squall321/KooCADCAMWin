// @lat: [[engine/skills#vent_slot_array]]

#include "vent_slot_array.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::vent_slot_array {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec library ─────────────────────────────────────────────────────────

namespace {

struct SpecEntry {
    const char* key;
    double      slot_length_mm;
    double      slot_width_mm;
    double      pitch_mm;
};

constexpr std::array<SpecEntry, 3> kSpecTable {{
    { "VS-S",  20.0, 2.0, 4.0 },   // small enclosure
    { "VS-M",  40.0, 3.0, 6.0 },   // medium
    { "VS-L",  60.0, 4.0, 8.0 },   // large
}};

}  // namespace

SlotSpec slotSpecFor(const std::string& key)
{
    for (const auto& e : kSpecTable) {
        if (key == e.key) return { e.slot_length_mm, e.slot_width_mm, e.pitch_mm };
    }
    return {};
}

namespace {

SlotSpec resolveSpec(const Input& in)
{
    if (!in.spec_key.empty()) return slotSpecFor(in.spec_key);
    return { in.slot_length_mm, in.slot_width_mm, in.pitch_mm };
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!in.spec_key.empty()) {
        SlotSpec s = slotSpecFor(in.spec_key);
        if (s.pitch_mm <= 0.0) {
            r.add("DFM-VENT-SP", "error",
                  "vent_slot_array: unknown spec_key '" + in.spec_key +
                  "' (supported: VS-S/VS-M/VS-L)");
            return r;
        }
    } else {
        if (in.slot_length_mm <= 0.0 || in.slot_width_mm <= 0.0 ||
            in.pitch_mm <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "vent_slot_array: spec_key empty AND "
                  "slot_length/width/pitch not all > 0");
            return r;
        }
    }
    if (in.slot_count < 2) {
        r.add("DFM-VENT-N", "error",
              "vent_slot_array: slot_count must be ≥ 2");
    }
    const SlotSpec eff = resolveSpec(in);
    if (eff.slot_width_mm > 0.0 && eff.slot_width_mm < 0.8) {
        r.add("DFM-VENT-WIDTH", "error",
              "vent_slot_array: slot_width " + std::to_string(eff.slot_width_mm) +
              " mm < 0.8 mm (DFM-002 derived)");
    }
    if (eff.pitch_mm > 0.0 && eff.slot_width_mm >= eff.pitch_mm) {
        r.add("DFM-VENT-FIT", "error",
              "vent_slot_array: slot_width >= pitch (adjacent slots would merge)");
    }

    // Auto-detect: query the workpiece bbox to size the face dimension along
    // the slot axis.  If the spec'd array span overruns it, fail FIT.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double maxBaseDim = std::max(xMax - xMin, yMax - yMin);
    if (in.slot_count >= 2 && eff.pitch_mm > 0.0) {
        const double span = (in.slot_count - 1) * eff.pitch_mm + eff.slot_width_mm;
        if (span > maxBaseDim + 1e-3) {
            r.add("DFM-VENT-FIT", "error",
                  "vent_slot_array: array span " + std::to_string(span) +
                  " mm > base max dim " + std::to_string(maxBaseDim) + " mm");
        }
    }

    // Free-area DFM gate: a vent that doesn't pass enough air is a defect.
    // We compute it relative to the bbox-projected area of the face along the
    // slot axis: rectangular envelope = slot_length × span.
    if (in.slot_count >= 2 && eff.slot_length_mm > 0.0 && eff.pitch_mm > 0.0) {
        const double span = (in.slot_count - 1) * eff.pitch_mm + eff.slot_width_mm;
        const double envelopeArea  = eff.slot_length_mm * span;
        const double openArea      = in.slot_count *
                                     eff.slot_length_mm * eff.slot_width_mm;
        const double freeFrac = openArea / std::max(1e-9, envelopeArea);
        if (freeFrac < 0.50) {
            r.add("DFM-VENT-FA", "error",
                  "vent_slot_array: free area " +
                  std::to_string(freeFrac * 100.0) +
                  "% < 50 % of bbox face (airflow insufficient)");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "vent_slot_array DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const SlotSpec eff = resolveSpec(in);

    auto faceId = wp.resolve(in.face);
    if (!faceId) throw SkillError("vent_slot_array: face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zTop = zMax;
    const double wallThk = (in.wall_thickness_mm > 0.0) ? in.wall_thickness_mm
                                                        : 3.0;
    constexpr double kOverhang = 0.1;

    // Local axes: slot_axis = along slot length, pitchDir = perpendicular in XY.
    const gp_Dir sax = in.slot_axis;
    const gp_Dir pitchDir(-sax.Y(), sax.X(), 0.0);

    // Build N slot cutters, fuse them into a single cutter, then ONE cut.
    std::vector<TopoDS_Shape> slots;
    slots.reserve(in.slot_count);
    const double arrayHalfSpan =
        ((in.slot_count - 1) * eff.pitch_mm) / 2.0;

    for (int i = 0; i < in.slot_count; ++i) {
        const double pOffset = i * eff.pitch_mm - arrayHalfSpan;
        // Place origin at the slot's left-back-bottom corner (relative to
        // sax = +X local, pitchDir = +Y local, +Z local).  With V=+Z and
        // Vx=sax, box extrudes +X by slot_length, +pitchDir by slot_width,
        // +Z by depth — all from origin.  Origin Z is BELOW the wall so the
        // box extends upward through it.
        const gp_Pnt origin(
            in.center_x_mm
              - eff.slot_length_mm / 2.0 * sax.X()
              + pOffset                 * pitchDir.X()
              - eff.slot_width_mm  / 2.0 * pitchDir.X(),
            in.center_y_mm
              - eff.slot_length_mm / 2.0 * sax.Y()
              + pOffset                 * pitchDir.Y()
              - eff.slot_width_mm  / 2.0 * pitchDir.Y(),
            zTop - wallThk - kOverhang);
        // gp_Ax2(P, V, Vx): V = LOCAL +Z, Vx = sax.  Local Y = V × Vx =
        // +Z × sax = pitchDir (which is (-sax.Y, sax.X, 0) — perpendicular
        // in XY).
        const gp_Ax2 ax(origin, gp_Dir(0.0, 0.0, 1.0), sax);
        // box(ax, dx, dy, dz): dx along sax (length), dy along pitchDir
        // (width), dz along +Z (depth = through wall + 2 × overhang).
        slots.push_back(pr::box(ax,
            eff.slot_length_mm,
            eff.slot_width_mm,
            wallThk + 2.0 * kOverhang));
    }

    TopoDS_Shape cutter = slots.front();
    for (size_t i = 1; i < slots.size(); ++i) {
        cutter = pr::fuse(cutter, slots[i]);
    }
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double removedMm3 = in.slot_count *
        eff.slot_length_mm * eff.slot_width_mm * wallThk;

    json params = {
        { "face_id",          *faceId },
        { "slot_count",       in.slot_count },
        { "spec_key",         in.spec_key },
        { "slot_length_mm",   eff.slot_length_mm },
        { "slot_width_mm",    eff.slot_width_mm },
        { "pitch_mm",         eff.pitch_mm },
        { "slot_axis",        { sax.X(), sax.Y(), sax.Z() } },
        { "center_x_mm",      in.center_x_mm },
        { "center_y_mm",      in.center_y_mm },
        { "wall_thickness_mm", wallThk },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  in.slot_count },
        { "spec_key",          in.spec_key },
        { "slot_length_mm",    eff.slot_length_mm },
        { "slot_width_mm",     eff.slot_width_mm },
        { "pitch_mm",          eff.pitch_mm },
        { "slot_axis",         { sax.X(), sax.Y(), sax.Z() } },
        { "free_area_mm2",     in.slot_count *
                                 eff.slot_length_mm * eff.slot_width_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "end_mill;punch";
    tooling.tool_dia_mm      = eff.slot_width_mm;
    tooling.tool_length_mm   = wallThk + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = removedMm3;
    tooling.est_cycle_time_s  = std::max(3.0, in.slot_count * 1.5);
    tooling.extra = {
        { "operation_note",
          "Slot-mill with through-cut OR sheet-metal punch with slot die." },
        { "slot_count", in.slot_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::vent_slot_array applied: spec={} count={} ({}×{} mm, pitch {})",
                  in.spec_key, in.slot_count,
                  eff.slot_length_mm, eff.slot_width_mm, eff.pitch_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic for vent arrays is unreliable (slots can be confused
// with mill_slot single-cuts).  We use the metadata-replay fallback.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "slot_count",      f.params.value("slot_count", 0) },
            { "spec_key",        f.params.value("spec_key",   std::string()) },
            { "slot_length_mm",  f.params.value("slot_length_mm", 0.0) },
            { "slot_width_mm",   f.params.value("slot_width_mm",  0.0) },
            { "pitch_mm",        f.params.value("pitch_mm",       0.0) },
            { "slot_axis",       f.params.value("slot_axis",
                                   json::array({1.0,0.0,0.0})) },
            { "center_x_mm",     f.params.value("center_x_mm",    0.0) },
            { "center_y_mm",     f.params.value("center_y_mm",    0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.90, json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::vent_slot_array
