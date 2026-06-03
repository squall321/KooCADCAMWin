// @lat: [[engine/skills#bspp_g_port]]

#include "bspp_g_port.hpp"

#include "Workpiece.hpp"
#include "_hydraulic_ports_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::bspp_g_port {

namespace pr = koocadcam::engine::prim;
namespace ht = koocadcam::skill::hydraulic_ports;
using nlohmann::json;

namespace {
constexpr double kOverhang = 0.05;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bspp_g_port: depth_mm must be > 0");
    }
    if (in.g_size.empty()) {
        r.add("DFM-BSPP-CODE", "error", "bspp_g_port: g_size is empty");
        return r;
    }
    const ht::BsppSpec* s = ht::findBspp(in.g_size);
    if (!s) {
        r.add("DFM-BSPP-CODE", "error",
              "bspp_g_port: unknown g_size '" + in.g_size +
              "' (supported: G1/8, G1/4, G3/8, G1/2, G3/4, G1)");
        return r;
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zSpan = zMax - zMin;
    if (zSpan < in.depth_mm + 2.0) {
        r.add("DFM-BSPP-DEPTH", "error",
              "bspp_g_port: face thickness " + std::to_string(zSpan) +
              " mm < depth + 2 mm safety");
    }
    const double margin = s->thread_od_mm / 2.0 + 1.0;
    if (in.position_x_mm - xMin < margin || xMax - in.position_x_mm < margin ||
        in.position_y_mm - yMin < margin || yMax - in.position_y_mm < margin)
    {
        r.add("DFM-BSPP-XY", "error",
              "bspp_g_port: position within " + std::to_string(margin) +
              " mm of face edge");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bspp_g_port DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ht::BsppSpec* s = ht::findBspp(in.g_size);
    if (!s) throw SkillError("bspp_g_port: g_size lookup failed unexpectedly");

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError("bspp_g_port: face_id datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const gp_Dir adir = in.axis_dir;
    gp_Pnt entryStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        entryStart = gp_Pnt(in.position_x_mm, in.position_y_mm,
                            (adir.Z() < 0) ? zMax + kOverhang : zMin - kOverhang);
    } else {
        entryStart = gp_Pnt(in.position_x_mm, in.position_y_mm,
                            (zMin + zMax) / 2.0 - adir.Z() * kOverhang);
    }

    const double tapR    = s->tap_drill_dia_mm / 2.0;
    const double threadR = s->thread_od_mm     / 2.0;
    const double chamfLeg = s->chamfer_leg_mm;
    const double reliefDepth = chamfLeg + s->pitch_mm * 0.5;

    // ── Sub-cut #1: main thread bore (tap drill dia for the full depth) ──
    const gp_Ax2 boreAx(entryStart, adir);
    const TopoDS_Shape mainBore =
        pr::cylinder(boreAx, tapR, in.depth_mm + kOverhang);

    // ── Sub-cut #2: face counterbore relief at the mouth ─────────────────
    // Wider than thread OD by 0.5 mm radial, depth = reliefDepth.
    const double reliefR = threadR + 0.5;
    const gp_Ax2 reliefAx(entryStart, adir);
    const TopoDS_Shape relief =
        pr::cylinder(reliefAx, reliefR, reliefDepth + kOverhang);

    // ── Sub-cut #3: 45° entry chamfer (cone frustum, leg = 0.5 × pitch) ──
    const gp_Ax2 chamfAx(entryStart, adir);
    const TopoDS_Shape chamf = pr::coneFrustum(
        chamfAx,
        reliefR + chamfLeg,           // wide at the face
        reliefR,                       // narrows to relief wall
        chamfLeg);

    TopoDS_Shape unified = pr::fuse(mainBore, relief);
    unified = pr::fuse(unified, chamf);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), unified);

    const double volBore   = M_PI * tapR * tapR * in.depth_mm;
    const double volRelief = M_PI * (reliefR * reliefR - tapR * tapR) * reliefDepth;
    const double volChamf  = M_PI / 3.0 * chamfLeg
                           * (reliefR * reliefR + reliefR * (reliefR + chamfLeg)
                              + (reliefR + chamfLeg) * (reliefR + chamfLeg))
                           - M_PI * reliefR * reliefR * chamfLeg;
    const double volRemoved = volBore + std::max(0.0, volRelief) + std::max(0.0, volChamf);

    json params = {
        { "face_id_resolved", *faceId },
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "g_size",           in.g_size },
        { "depth_mm",         in.depth_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "port_standard",          "BSPP_ISO_228" },
        { "size_code",              in.g_size },
        { "subfeature_count",       3 },
        { "thread_od_mm",           s->thread_od_mm },
        { "tap_drill_dia_mm",       s->tap_drill_dia_mm },
        { "pitch_mm",               s->pitch_mm },
        { "chamfer_leg_mm",         chamfLeg },
        { "derived_volume_removed", volRemoved },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;counterbore;chamfer_mill";
    tooling.tool_dia_mm       = s->tap_drill_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.est_cycle_time_s  = std::max(8.0, in.depth_mm * 0.4 + 4.0);
    tooling.stock_removed_mm3 = volRemoved;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", s->tap_drill_dia_mm },
              { "role", "tap_pilot_bore" } },
            { { "tool_type", "counterbore" },
              { "tool_dia_mm", 2.0 * reliefR },
              { "depth_mm", reliefDepth },
              { "role", "face_relief" } },
            { { "tool_type", "chamfer_mill" },
              { "leg_mm", chamfLeg },
              { "angle_deg", 45.0 },
              { "role", "thread_start_chamfer" } },
        } },
        { "note", "BSPP / ISO 228-1 G-thread parallel hydraulic port" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::bspp_g_port applied: {} OD={} tap={} depth={}",
                  in.g_size, s->thread_od_mm, s->tap_drill_dia_mm, in.depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::bspp_g_port
