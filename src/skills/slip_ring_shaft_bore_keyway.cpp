// @lat: [[engine/skills#slip_ring_shaft_bore_keyway]]

#include "slip_ring_shaft_bore_keyway.hpp"

#include "Workpiece.hpp"
#include "_keyway_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::slip_ring_shaft_bore_keyway {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.shaft_bore_dia_mm <= 0.0 || in.key_length_mm <= 0.0 ||
        in.key_position_z_mm < 0.0 ||
        in.cable_channel_width_mm <= 0.0 ||
        in.cable_channel_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "slip_ring_shaft_bore_keyway: all dims must be > 0");
        return r;
    }

    const auto* band = keyway::findDin6885Band(in.shaft_bore_dia_mm);
    if (!band) {
        r.add("DFM-KEYWAY", "error",
              "slip_ring_shaft_bore_keyway: shaft_bore_dia " +
              std::to_string(in.shaft_bore_dia_mm) +
              " mm has no DIN 6885 parallel-key band");
    }

    // Keyway must fit within the bore depth.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double boreDepth = zMax - zMin;
    if (in.key_position_z_mm + in.key_length_mm > boreDepth) {
        r.add("DFM-KEY-POS", "error",
              "slip_ring_shaft_bore_keyway: key_position_z + key_length (" +
              std::to_string(in.key_position_z_mm + in.key_length_mm) +
              " mm) exceeds bore depth " + std::to_string(boreDepth) + " mm");
    }

    // Cable channel must be narrower than the bore (else it would breach it).
    if (in.cable_channel_width_mm >= in.shaft_bore_dia_mm) {
        r.add("DFM-CHANNEL", "error",
              "slip_ring_shaft_bore_keyway: cable_channel_width " +
              std::to_string(in.cable_channel_width_mm) +
              " mm >= shaft bore dia " +
              std::to_string(in.shaft_bore_dia_mm) + " mm");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "slip_ring_shaft_bore_keyway DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* band = keyway::findDin6885Band(in.shaft_bore_dia_mm);
    // band guaranteed non-null by validate.

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx   = in.axis_origin.X();
    const double cy   = in.axis_origin.Y();

    const double boreR     = in.shaft_bore_dia_mm / 2.0;
    const double boreDepth = (zMax - zMin) + 2.0 * kOver;

    // ── 1) Central shaft bore (through, down from top) ───────────────────
    const gp_Pnt boreStart(cx, cy, zMin - kOver);
    const gp_Ax2 boreAx(boreStart, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::cylinder(boreAx, boreR, boreDepth));

    // ── 2) DIN 6885 keyway (box pocket on the +X bore wall) ──────────────
    // Keyway width b along Y, depth t2 added radially outward into the hub
    // bore wall, length along Z (axial).  Placed at +X side of the bore.
    const double keyW = band->key_width_mm;
    const double keyDepth = band->hub_keyway_depth_mm;  // t2 into the bore wall
    const gp_Pnt keyOrigin(
        cx + boreR - kOver,                  // start at the bore wall
        cy - keyW / 2.0,
        topZ - in.key_position_z_mm - in.key_length_mm);
    const TopoDS_Shape keyTool = pr::box(
        gp_Ax2(keyOrigin, gp::DZ()),
        keyDepth + kOver, keyW, in.key_length_mm);
    current = pr::cut(current, keyTool);

    // ── 3) Cable routing channel (box pocket along top face, +Y) ─────────
    const double chanLenY = (yMax - cy) + kOver;  // from center to +Y edge
    const gp_Pnt chanOrigin(
        cx - in.cable_channel_width_mm / 2.0,
        cy,
        topZ - in.cable_channel_depth_mm);
    const TopoDS_Shape chanTool = pr::box(
        gp_Ax2(chanOrigin, gp::DZ()),
        in.cable_channel_width_mm, chanLenY, in.cable_channel_depth_mm + kOver);
    current = pr::cut(current, chanTool);

    // ── Derived volume (analytic) ────────────────────────────────────────
    const double vBore = M_PI * boreR * boreR * (zMax - zMin);
    const double vKey  = keyDepth * keyW * in.key_length_mm;
    const double vChan = in.cable_channel_width_mm * chanLenY *
                         in.cable_channel_depth_mm;
    const double volRemoved = vBore + vKey + vChan;

    json params = {
        { "axis_origin",            { in.axis_origin.X(),
                                      in.axis_origin.Y(),
                                      in.axis_origin.Z() } },
        { "shaft_bore_dia_mm",      in.shaft_bore_dia_mm },
        { "key_position_z_mm",      in.key_position_z_mm },
        { "key_length_mm",          in.key_length_mm },
        { "cable_channel_width_mm", in.cable_channel_width_mm },
        { "cable_channel_depth_mm", in.cable_channel_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "wind_feature_type",          "slip_ring_shaft_bore_keyway" },
        { "subfeature_count",           3 },
        { "shaft_bore_dia_mm",          in.shaft_bore_dia_mm },
        { "derived_key_width_mm",       band->key_width_mm },
        { "derived_key_height_mm",      band->key_height_mm },
        { "derived_hub_keyway_depth_mm",band->hub_keyway_depth_mm },
        { "derived_key_tolerance",      band->tolerance_grade },
        { "cable_channel_width_mm",     in.cable_channel_width_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "DIN 6885 Part A" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;keyseat_cutter;slot_mill";
    tooling.tool_dia_mm       = in.shaft_bore_dia_mm;
    tooling.tool_length_mm    = (zMax - zMin) + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 170.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 100.0;
    tooling.extra = {
        { "wind_feature_type", "slip_ring_shaft_bore_keyway" },
        { "key_standard",      "DIN 6885 Part A" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::slip_ring_shaft_bore_keyway: bore Ø{} keyW={} chan={}",
                  in.shaft_bore_dia_mm, band->key_width_mm,
                  in.cable_channel_width_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",            "metadata_replay" },
            { "is_compound",       true },
            { "wind_feature_type", "slip_ring_shaft_bore_keyway" },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: a sizeable +Z central bore cylinder.
    double maxR = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.Z()) < 0.9) continue;
            const double radius = s.Cylinder().Radius();
            if (radius > maxR) maxR = radius;
        } catch (...) {}
    }
    if (maxR >= 10.0) {
        json recovered = { { "shaft_bore_dia_mm", 2.0 * maxR } };
        json matched   = { { "source",     "geometric_central_bore" },
                           { "max_radius", maxR } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::slip_ring_shaft_bore_keyway
