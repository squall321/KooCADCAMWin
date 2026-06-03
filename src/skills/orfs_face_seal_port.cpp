// @lat: [[engine/skills#orfs_face_seal_port]]

#include "orfs_face_seal_port.hpp"

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

namespace koocadcam::skill::orfs_face_seal_port {

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
        r.add("DFM-INPUT", "error", "orfs_face_seal_port: depth_mm must be > 0");
    }
    if (in.orfs_dash.empty()) {
        r.add("DFM-ORFS-CODE", "error", "orfs_face_seal_port: orfs_dash is empty");
        return r;
    }
    const ht::OrfsSpec* s = ht::findOrfs(in.orfs_dash);
    if (!s) {
        r.add("DFM-ORFS-CODE", "error",
              "orfs_face_seal_port: unknown orfs_dash '" + in.orfs_dash +
              "' (supported: -04, -06, -08, -12, -16, -20)");
        return r;
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zSpan = zMax - zMin;
    if (zSpan < in.depth_mm + 2.0) {
        r.add("DFM-ORFS-DEPTH", "error",
              "orfs_face_seal_port: face thickness " + std::to_string(zSpan) +
              " mm < depth + 2 mm safety");
    }
    const double margin = s->oring_groove_od_mm / 2.0 + 1.0;
    if (in.position_x_mm - xMin < margin || xMax - in.position_x_mm < margin ||
        in.position_y_mm - yMin < margin || yMax - in.position_y_mm < margin)
    {
        r.add("DFM-ORFS-XY", "error",
              "orfs_face_seal_port: position within " + std::to_string(margin) +
              " mm of face edge");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "orfs_face_seal_port DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ht::OrfsSpec* s = ht::findOrfs(in.orfs_dash);
    if (!s) throw SkillError("orfs_face_seal_port: orfs_dash lookup failed unexpectedly");

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError("orfs_face_seal_port: face_id datum unresolved");

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

    const double threadR = s->thread_od_mm / 2.0;
    const double boreR   = s->bore_dia_mm  / 2.0;
    const double grID    = s->oring_groove_id_mm / 2.0;
    const double grOD    = s->oring_groove_od_mm / 2.0;
    const double grD     = s->oring_groove_depth_mm;
    const double chamfLeg = std::max(0.4, s->pitch_mm * 0.5);

    // ── Sub-cut #1: UN/UNF straight thread bore (deepest) ────────────────
    const gp_Ax2 threadAx(entryStart, adir);
    const TopoDS_Shape threadBore =
        pr::cylinder(threadAx, threadR, in.depth_mm + kOverhang);

    // ── Sub-cut #2: flow bore (smaller than thread, full depth) ──────────
    // This is the actual fluid passage; thread bore is wider as it accepts
    // the UN nut.  In the modelled geometry both contribute material removal.
    const gp_Ax2 boreAx(entryStart, adir);
    const TopoDS_Shape flowBore =
        pr::cylinder(boreAx, boreR, in.depth_mm + kOverhang);

    // ── Sub-cut #3: circular face O-ring groove around the bore ──────────
    // grID..grOD radii; groove sits at the +adir face, depth = grD.
    const gp_Ax2 grooveAx(entryStart, adir);
    const TopoDS_Shape oringGroove =
        pr::annularRing(grooveAx, grOD, grID, grD + kOverhang);

    // ── Sub-cut #4: light entry chamfer at the thread mouth ──────────────
    const gp_Ax2 chamfAx(entryStart, adir);
    const TopoDS_Shape chamf = pr::coneFrustum(
        chamfAx,
        threadR + chamfLeg,
        threadR,
        chamfLeg);

    TopoDS_Shape unified = pr::fuse(threadBore, flowBore);
    unified = pr::fuse(unified, oringGroove);
    unified = pr::fuse(unified, chamf);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), unified);

    const double volThread = M_PI * threadR * threadR * in.depth_mm;
    const double volGroove = M_PI * (grOD * grOD - grID * grID) * grD;
    const double volChamf  = M_PI / 3.0 * chamfLeg
                           * (threadR * threadR + threadR * (threadR + chamfLeg)
                              + (threadR + chamfLeg) * (threadR + chamfLeg))
                           - M_PI * threadR * threadR * chamfLeg;
    // flowBore is fully contained inside threadBore (boreR < threadR) so
    // it contributes nothing extra; counted in volThread.
    const double volRemoved = volThread + volGroove + std::max(0.0, volChamf);

    json params = {
        { "face_id_resolved", *faceId },
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "orfs_dash",        in.orfs_dash },
        { "depth_mm",         in.depth_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "port_standard",          "ORFS_SAE_J1453" },
        { "size_code",              in.orfs_dash },
        { "subfeature_count",       4 },
        { "thread_od_mm",           s->thread_od_mm },
        { "pitch_mm",               s->pitch_mm },
        { "bore_dia_mm",            s->bore_dia_mm },
        { "oring_groove_id_mm",     s->oring_groove_id_mm },
        { "oring_groove_od_mm",     s->oring_groove_od_mm },
        { "oring_groove_depth_mm",  s->oring_groove_depth_mm },
        { "chamfer_leg_mm",         chamfLeg },
        { "derived_volume_removed", volRemoved },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;drill;groove_mill;chamfer_mill";
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
              { "role", "UN_thread_bore" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", s->bore_dia_mm },
              { "role", "flow_passage" } },
            { { "tool_type", "groove_mill" },
              { "id_dia_mm", s->oring_groove_id_mm },
              { "od_dia_mm", s->oring_groove_od_mm },
              { "depth_mm", s->oring_groove_depth_mm },
              { "role", "face_o_ring_groove" } },
            { { "tool_type", "chamfer_mill" },
              { "leg_mm", chamfLeg },
              { "angle_deg", 45.0 } },
        } },
        { "note", "ORFS face-seal port per SAE J1453" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::orfs_face_seal_port applied: {} OD={} bore={} depth={}",
                  in.orfs_dash, s->thread_od_mm, s->bore_dia_mm, in.depth_mm);

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

}  // namespace koocadcam::skill::orfs_face_seal_port
