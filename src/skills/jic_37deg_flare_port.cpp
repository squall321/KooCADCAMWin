// @lat: [[engine/skills#jic_37deg_flare_port]]

#include "jic_37deg_flare_port.hpp"

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

namespace koocadcam::skill::jic_37deg_flare_port {

namespace pr = koocadcam::engine::prim;
namespace ht = koocadcam::skill::hydraulic_ports;
using nlohmann::json;

namespace {
constexpr double kOverhang = 0.05;
// 37° half-angle (74° included) flare seat per SAE J514.
constexpr double kJicHalfAngleDeg = 37.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "jic_37deg_flare_port: depth_mm must be > 0");
    }
    if (in.jic_dash.empty()) {
        r.add("DFM-JIC-CODE", "error", "jic_37deg_flare_port: jic_dash is empty");
        return r;
    }
    const ht::JicSpec* s = ht::findJic(in.jic_dash);
    if (!s) {
        r.add("DFM-JIC-CODE", "error",
              "jic_37deg_flare_port: unknown jic_dash '" + in.jic_dash +
              "' (supported: -04, -06, -08, -12)");
        return r;
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zSpan = zMax - zMin;
    if (zSpan < in.depth_mm + 2.0) {
        r.add("DFM-JIC-DEPTH", "error",
              "jic_37deg_flare_port: face thickness " + std::to_string(zSpan) +
              " mm < depth + 2 mm safety");
    }
    const double margin = s->thread_od_mm / 2.0 + 1.0;
    if (in.position_x_mm - xMin < margin || xMax - in.position_x_mm < margin ||
        in.position_y_mm - yMin < margin || yMax - in.position_y_mm < margin)
    {
        r.add("DFM-JIC-XY", "error",
              "jic_37deg_flare_port: position within " + std::to_string(margin) +
              " mm of face edge");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "jic_37deg_flare_port DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ht::JicSpec* s = ht::findJic(in.jic_dash);
    if (!s) throw SkillError("jic_37deg_flare_port: jic_dash lookup failed unexpectedly");

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError("jic_37deg_flare_port: face_id datum unresolved");

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

    const double threadR  = s->thread_od_mm / 2.0;
    const double flareR   = s->flare_od_mm / 2.0;
    const double minR     = s->flare_min_dia_mm / 2.0;
    const double seatD    = s->seat_depth_mm;

    // ── Sub-cut #1: UNF straight thread bore (full depth) ────────────────
    // Drill at thread MAJOR dia for the modelled straight bore (no thread
    // form modelling).  Real CAM step is tap; here we cut the cylindrical
    // envelope.
    const gp_Ax2 boreAx(entryStart, adir);
    const TopoDS_Shape threadBore =
        pr::cylinder(boreAx, threadR, in.depth_mm + kOverhang);

    // ── Sub-cut #2: 37° conical flare seat at the mouth ─────────────────
    // Wide end at the face = flareR; narrow end deeper = minR; depth = seatD.
    const gp_Ax2 seatAx(entryStart, adir);
    const TopoDS_Shape flareSeat =
        pr::coneFrustum(seatAx, flareR, minR, seatD + kOverhang);

    // ── Sub-cut #3: inner flow-passage bore (smaller dia, full depth) ────
    // This is the central fluid passage through the fitting; minR keeps the
    // flow area below the seat.
    const gp_Ax2 flowAx(entryStart, adir);
    const TopoDS_Shape flowBore =
        pr::cylinder(flowAx, minR, in.depth_mm + kOverhang);

    TopoDS_Shape unified = pr::fuse(threadBore, flareSeat);
    unified = pr::fuse(unified, flowBore);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), unified);

    // Derived removed volume = union of (threadBore ∪ flareSeat ∪ flowBore).
    // Geometrically:
    //   • threadBore = cylinder of radius threadR over depth_mm (largest).
    //   • flareSeat = cone frustum (flareR→minR) over seatD (flareR < threadR).
    //   • flowBore  = cylinder of radius minR over depth_mm (minR < threadR).
    // The flare seat and flow bore are fully contained inside the thread
    // bore radius-wise; over the threadBore's depth, no additional volume
    // is removed.  Volume removed = threadBore volume.
    const double volThread = M_PI * threadR * threadR * in.depth_mm;
    const double volRemoved = volThread;

    json params = {
        { "face_id_resolved", *faceId },
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "jic_dash",         in.jic_dash },
        { "depth_mm",         in.depth_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "port_standard",          "JIC_37_SAE_J514" },
        { "size_code",              in.jic_dash },
        { "subfeature_count",       3 },
        { "thread_od_mm",           s->thread_od_mm },
        { "pitch_mm",               s->pitch_mm },
        { "flare_od_mm",            s->flare_od_mm },
        { "flare_min_dia_mm",       s->flare_min_dia_mm },
        { "seat_depth_mm",          seatD },
        { "flare_half_angle_deg",   kJicHalfAngleDeg },
        { "derived_volume_removed", volRemoved },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;flare_form;drill";
    tooling.tool_dia_mm       = s->thread_od_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.est_cycle_time_s  = std::max(10.0, in.depth_mm * 0.6 + 5.0);
    tooling.stock_removed_mm3 = volRemoved;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", s->thread_od_mm },
              { "role", "UNF_thread_bore" } },
            { { "tool_type", "flare_form" },
              { "od_dia_mm", s->flare_od_mm },
              { "angle_deg", kJicHalfAngleDeg },
              { "depth_mm", seatD } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", s->flare_min_dia_mm },
              { "role", "flow_passage" } },
        } },
        { "note", "JIC 37° flare port (SAE J514 / ISO 8434-2)" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::jic_37deg_flare_port applied: {} OD={} flareOD={} depth={}",
                  in.jic_dash, s->thread_od_mm, s->flare_od_mm, in.depth_mm);

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

}  // namespace koocadcam::skill::jic_37deg_flare_port
