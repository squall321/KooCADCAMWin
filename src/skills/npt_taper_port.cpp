// @lat: [[engine/skills#npt_taper_port]]

#include "npt_taper_port.hpp"

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

namespace koocadcam::skill::npt_taper_port {

namespace pr = koocadcam::engine::prim;
namespace ht = koocadcam::skill::hydraulic_ports;
using nlohmann::json;

namespace {
constexpr double kOverhang = 0.05;
// Half-angle per side per ANSI NPT (1.7857° / ft total / 2).
constexpr double kNptHalfAngleDeg = 1.7899;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "npt_taper_port: depth_mm must be > 0");
    }
    if (in.npt_size.empty()) {
        r.add("DFM-NPT-CODE", "error", "npt_taper_port: npt_size is empty");
        return r;
    }
    const ht::NptSpec* s = ht::findNpt(in.npt_size);
    if (!s) {
        r.add("DFM-NPT-CODE", "error",
              "npt_taper_port: unknown npt_size '" + in.npt_size +
              "' (supported: 1/16, 1/8, 1/4, 3/8, 1/2, 3/4, 1)");
        return r;
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zSpan = zMax - zMin;
    if (zSpan < in.depth_mm + 2.0) {
        r.add("DFM-NPT-DEPTH", "error",
              "npt_taper_port: face thickness " + std::to_string(zSpan) +
              " mm < depth + 2 mm safety");
    }
    const double margin = s->major_dia_mm / 2.0 + 1.0;
    if (in.position_x_mm - xMin < margin || xMax - in.position_x_mm < margin ||
        in.position_y_mm - yMin < margin || yMax - in.position_y_mm < margin)
    {
        r.add("DFM-NPT-XY", "error",
              "npt_taper_port: position within " + std::to_string(margin) +
              " mm of face edge");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "npt_taper_port DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ht::NptSpec* s = ht::findNpt(in.npt_size);
    if (!s) throw SkillError("npt_taper_port: npt_size lookup failed unexpectedly");

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError("npt_taper_port: face_id datum unresolved");

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

    const double majorR = s->major_dia_mm / 2.0;
    const double l1     = s->l1_depth_mm;
    const double chamfLeg = s->chamfer_leg_mm;

    // Compute the small end of the cone at depth `in.depth_mm`.
    const double taperPerSide = std::tan(kNptHalfAngleDeg * M_PI / 180.0);
    const double smallR = std::max(0.1, majorR - taperPerSide * in.depth_mm);

    // ── Sub-cut #1: tapered bore (cone frustum) — apex angle per NPT ─────
    const gp_Ax2 coneAx(entryStart, adir);
    const TopoDS_Shape nptCone =
        pr::coneFrustum(coneAx, majorR, smallR, in.depth_mm + kOverhang);

    // ── Sub-cut #2: face counterbore relief at the mouth ─────────────────
    const double reliefR = majorR + 0.5;
    const double reliefD = 1.0;
    const gp_Ax2 reliefAx(entryStart, adir);
    const TopoDS_Shape relief =
        pr::cylinder(reliefAx, reliefR, reliefD + kOverhang);

    // ── Sub-cut #3: chamfered entry (45° bevel) ──────────────────────────
    const gp_Ax2 chamfAx(entryStart, adir);
    const TopoDS_Shape chamf = pr::coneFrustum(
        chamfAx,
        reliefR + chamfLeg,
        reliefR,
        chamfLeg);

    TopoDS_Shape unified = pr::fuse(nptCone, relief);
    unified = pr::fuse(unified, chamf);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), unified);

    const double volCone   = M_PI * in.depth_mm / 3.0
                           * (majorR * majorR + majorR * smallR + smallR * smallR);
    const double volRelief = M_PI * (reliefR * reliefR - majorR * majorR) * reliefD;
    const double volChamf  = M_PI / 3.0 * chamfLeg
                           * (reliefR * reliefR + reliefR * (reliefR + chamfLeg)
                              + (reliefR + chamfLeg) * (reliefR + chamfLeg))
                           - M_PI * reliefR * reliefR * chamfLeg;
    const double volRemoved = volCone + std::max(0.0, volRelief) + std::max(0.0, volChamf);

    json params = {
        { "face_id_resolved", *faceId },
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "npt_size",         in.npt_size },
        { "depth_mm",         in.depth_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "port_standard",          "NPT_ASME_B1_20_1" },
        { "size_code",              in.npt_size },
        { "subfeature_count",       3 },
        { "major_dia_mm",           s->major_dia_mm },
        { "minor_dia_mm",           s->minor_dia_mm },
        { "l1_depth_mm",            l1 },
        { "small_end_dia_mm",       2.0 * smallR },
        { "taper_per_side_deg",     kNptHalfAngleDeg },
        { "chamfer_leg_mm",         chamfLeg },
        { "derived_volume_removed", volRemoved },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "tapered_tap;counterbore;chamfer_mill";
    tooling.tool_dia_mm       = s->major_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 80.0;        // NPT taps run slow
    tooling.feed_per_tooth_mm = 0.0;         // synchronized feed
    tooling.est_cycle_time_s  = std::max(10.0, in.depth_mm * 1.0 + 5.0);
    tooling.stock_removed_mm3 = volRemoved;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "tapered_tap" },
              { "for", "NPT " + in.npt_size },
              { "l1_depth_mm", l1 },
              { "taper_deg_per_side", kNptHalfAngleDeg } },
            { { "tool_type", "counterbore" },
              { "tool_dia_mm", 2.0 * reliefR },
              { "depth_mm", reliefD } },
            { { "tool_type", "chamfer_mill" },
              { "leg_mm", chamfLeg },
              { "angle_deg", 45.0 } },
        } },
        { "note", "ANSI NPT taper port — slice-13 hydraulic" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::npt_taper_port applied: {} OD={} depth={}",
                  in.npt_size, s->major_dia_mm, in.depth_mm);

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

}  // namespace koocadcam::skill::npt_taper_port
