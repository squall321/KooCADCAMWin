// @lat: [[engine/skills#louvered_vent]]

#include "louvered_vent.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <BRepBuilderAPI_Transform.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::louvered_vent {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec library ─────────────────────────────────────────────────────────

namespace {

struct SpecEntry {
    const char* key;
    double      tilt_angle_deg;
    double      slot_length_mm;
    double      slot_width_mm;
    double      pitch_mm;
};

constexpr std::array<SpecEntry, 3> kSpecTable {{
    { "L-15", 15.0, 20.0, 3.0, 6.0 },    // standard
    { "L-30", 30.0, 30.0, 4.0, 8.0 },    // high airflow
    { "L-45", 45.0, 40.0, 5.0, 10.0 },   // max airflow
}};

}  // namespace

LouverSpec louverSpecFor(const std::string& key)
{
    for (const auto& e : kSpecTable) {
        if (key == e.key) {
            return { e.tilt_angle_deg, e.slot_length_mm,
                     e.slot_width_mm, e.pitch_mm };
        }
    }
    return {};
}

namespace {

LouverSpec resolveSpec(const Input& in)
{
    if (!in.spec_key.empty()) return louverSpecFor(in.spec_key);
    return { in.tilt_angle_deg, in.slot_length_mm,
             in.slot_width_mm, in.pitch_mm };
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!in.spec_key.empty()) {
        LouverSpec s = louverSpecFor(in.spec_key);
        if (s.pitch_mm <= 0.0) {
            r.add("DFM-LOUVER-SP", "error",
                  "louvered_vent: unknown spec_key '" + in.spec_key +
                  "' (supported: L-15/L-30/L-45)");
            return r;
        }
    } else {
        if (in.tilt_angle_deg <= 0.0 || in.slot_length_mm <= 0.0 ||
            in.slot_width_mm  <= 0.0 || in.pitch_mm       <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "louvered_vent: spec_key empty AND "
                  "tilt_angle/slot_length/width/pitch not all > 0");
            return r;
        }
    }
    if (in.louver_count < 2) {
        r.add("DFM-LOUVER-N", "error",
              "louvered_vent: louver_count must be ≥ 2");
    }
    const LouverSpec eff = resolveSpec(in);

    if (eff.tilt_angle_deg < 5.0 || eff.tilt_angle_deg > 75.0) {
        r.add("DFM-LOUVER-ANG", "error",
              "louvered_vent: tilt_angle " +
              std::to_string(eff.tilt_angle_deg) +
              "° outside [5°, 75°] (flat → ineffective; near-90° → blocked)");
    }
    if (eff.pitch_mm > 0.0 && eff.slot_width_mm >= eff.pitch_mm) {
        r.add("DFM-LOUVER-FIT", "error",
              "louvered_vent: slot_width >= pitch (adjacent louvers merge)");
    }

    // Auto-detect: use workpiece bbox max dim as available face length.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double maxBaseDim = std::max(xMax - xMin, yMax - yMin);

    if (in.louver_count >= 2 && eff.pitch_mm > 0.0) {
        const double span = (in.louver_count - 1) * eff.pitch_mm + eff.slot_width_mm;
        if (span > maxBaseDim + 1e-3) {
            r.add("DFM-LOUVER-FIT", "error",
                  "louvered_vent: array span " + std::to_string(span) +
                  " mm > base max dim " + std::to_string(maxBaseDim) + " mm");
        }
    }

    // Free-area: projection of louver openings as seen from front (axial).
    // The PROJECTED open area of one tilted slot is
    //   slot_length × (slot_width × cos(tilt)),
    // because the slot face tilts away from the projection direction.
    if (in.louver_count >= 2 && eff.slot_length_mm > 0.0 && eff.pitch_mm > 0.0) {
        const double span = (in.louver_count - 1) * eff.pitch_mm + eff.slot_width_mm;
        const double envelopeArea = eff.slot_length_mm * span;
        const double effectiveWidth =
            eff.slot_width_mm * std::cos(eff.tilt_angle_deg * M_PI / 180.0);
        const double openArea = in.louver_count *
            eff.slot_length_mm * effectiveWidth;
        const double freeFrac = openArea / std::max(1e-9, envelopeArea);
        if (freeFrac < 0.50) {
            r.add("DFM-LOUVER-FA", "error",
                  "louvered_vent: projected free area " +
                  std::to_string(freeFrac * 100.0) +
                  "% < 50 % of bbox face");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "louvered_vent DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const LouverSpec eff = resolveSpec(in);

    auto faceId = wp.resolve(in.face);
    if (!faceId) throw SkillError("louvered_vent: face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zTop = zMax;
    const double wallThk = (in.wall_thickness_mm > 0.0) ? in.wall_thickness_mm
                                                        : 3.0;
    constexpr double kOverhang = 0.2;

    const gp_Dir sax = in.slot_axis;
    const gp_Dir pitchDir(-sax.Y(), sax.X(), 0.0);

    // Build tilted slot cutters.  We build each as an axis-aligned thin box,
    // then rotate it about a horizontal axis (parallel to sax) passing
    // through the slot center so the box becomes tilted by tilt_angle.
    const double tiltRad = eff.tilt_angle_deg * M_PI / 180.0;
    // Cutter depth must extend through the tilted wall; cosθ shrinks the
    // through-thickness component, so allow extra depth proportional to 1/cos.
    const double cutterDepth =
        (wallThk + 2.0 * kOverhang) / std::max(0.2, std::cos(tiltRad));

    std::vector<TopoDS_Shape> slots;
    slots.reserve(in.louver_count);
    const double arrayHalfSpan = ((in.louver_count - 1) * eff.pitch_mm) / 2.0;

    for (int i = 0; i < in.louver_count; ++i) {
        const double pOffset = i * eff.pitch_mm - arrayHalfSpan;
        const gp_Pnt slotCenter(
            in.center_x_mm + pOffset * pitchDir.X(),
            in.center_y_mm + pOffset * pitchDir.Y(),
            zTop);
        // Build an upright box of length slot_length, width slot_width,
        // height cutterDepth, with V=+Z so it extrudes upward FROM origin
        // through the wall.  Origin is below the wall bottom by overhang.
        const gp_Pnt origin(
            slotCenter.X() - eff.slot_length_mm / 2.0 * sax.X()
                           - eff.slot_width_mm  / 2.0 * pitchDir.X(),
            slotCenter.Y() - eff.slot_length_mm / 2.0 * sax.Y()
                           - eff.slot_width_mm  / 2.0 * pitchDir.Y(),
            slotCenter.Z() - cutterDepth + kOverhang);
        const gp_Ax2 ax(origin, gp_Dir(0.0, 0.0, 1.0), sax);
        TopoDS_Shape uprightBox =
            pr::box(ax, eff.slot_length_mm, eff.slot_width_mm, cutterDepth);

        // Rotate it about the axis through slotCenter parallel to sax,
        // by tilt_angle.  After this transform, the box is "leaning" along
        // the pitch direction — i.e. a real louver tilt.
        gp_Trsf tr;
        tr.SetRotation(gp_Ax1(slotCenter, sax), tiltRad);
        BRepBuilderAPI_Transform xf(uprightBox, tr, true /*copy*/);
        slots.push_back(xf.Shape());
    }

    TopoDS_Shape cutter = slots.front();
    for (size_t i = 1; i < slots.size(); ++i) {
        cutter = pr::fuse(cutter, slots[i]);
    }
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double removedMm3 = in.louver_count *
        eff.slot_length_mm * eff.slot_width_mm * wallThk;

    json params = {
        { "face_id",          *faceId },
        { "louver_count",     in.louver_count },
        { "spec_key",         in.spec_key },
        { "tilt_angle_deg",   eff.tilt_angle_deg },
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
        { "subfeature_count",  in.louver_count },
        { "spec_key",          in.spec_key },
        { "tilt_angle_deg",    eff.tilt_angle_deg },
        { "slot_length_mm",    eff.slot_length_mm },
        { "slot_width_mm",     eff.slot_width_mm },
        { "pitch_mm",          eff.pitch_mm },
        { "slot_axis",         { sax.X(), sax.Y(), sax.Z() } },
        { "projected_free_area_mm2",
            in.louver_count * eff.slot_length_mm *
            (eff.slot_width_mm * std::cos(eff.tilt_angle_deg * M_PI / 180.0)) },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "louver_die;sheet_lance";
    tooling.tool_dia_mm      = eff.slot_width_mm;
    tooling.tool_length_mm   = wallThk + 5.0;
    tooling.tool_material    = "die_steel";
    tooling.flute_count      = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = removedMm3;
    tooling.est_cycle_time_s  = std::max(2.0, in.louver_count * 0.5);
    tooling.extra = {
        { "operation_note",
          "Punch + lance-form on press brake for sheet metal; 5-axis "
          "mill for thick-wall extrusion." },
        { "louver_count", in.louver_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::louvered_vent applied: spec={} count={} tilt={}°",
                  in.spec_key, in.louver_count, eff.tilt_angle_deg);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Tilted angular cuts are very hard to reliably distinguish geometrically
// without a planar-region clustering step.  We fall back to metadata replay.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "louver_count",   f.params.value("louver_count", 0) },
            { "spec_key",       f.params.value("spec_key",     std::string()) },
            { "tilt_angle_deg", f.params.value("tilt_angle_deg", 0.0) },
            { "slot_length_mm", f.params.value("slot_length_mm", 0.0) },
            { "slot_width_mm",  f.params.value("slot_width_mm",  0.0) },
            { "pitch_mm",       f.params.value("pitch_mm",       0.0) },
            { "slot_axis",      f.params.value("slot_axis",
                                   json::array({1.0,0.0,0.0})) },
            { "center_x_mm",    f.params.value("center_x_mm",    0.0) },
            { "center_y_mm",    f.params.value("center_y_mm",    0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.88, json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::louvered_vent
