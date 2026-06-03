// @lat: [[engine/skills#npt_taper_port_b120]]

#include "npt_taper_port_b120.hpp"

#include "Workpiece.hpp"
#include "_hydraulic_ports.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::npt_taper_port_b120 {

namespace pr = koocadcam::engine::prim;
namespace ht = koocadcam::skill::hyd_ports;
using nlohmann::json;

namespace {
constexpr double kOverhang = 0.05;
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thread_size.empty()) {
        r.add("DFM-NPT-CODE", "error",
              "npt_taper_port_b120: thread_size is empty");
        return r;
    }
    const ht::NptSpec* s = ht::findNpt(in.thread_size);
    if (!s) {
        r.add("DFM-NPT-CODE", "error",
              "npt_taper_port_b120: unknown thread_size '" + in.thread_size +
              "' (supported: 1/8, 1/4, 3/8, 1/2, 3/4)");
        return r;
    }

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) {
        r.add("DFM-NPT-FACE", "error",
              "npt_taper_port_b120: face_id datum unresolved");
        return r;
    }
    if (!wp.isFacePlanar(*faceId)) {
        r.add("DFM-NPT-FACE", "error",
              "npt_taper_port_b120: target face is not planar");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double minSpan = 1.5 * s->pitch_dia_at_E1_mm;
    if (xMax - xMin < minSpan || yMax - yMin < minSpan) {
        r.add("DFM-NPT-MAT", "error",
              "npt_taper_port_b120: face span < 1.5 × pitch_dia (need " +
              std::to_string(minSpan) + " mm)");
    }
    if (zMax - zMin < s->depth_mm + 2.0) {
        r.add("DFM-NPT-DEPTH", "error",
              "npt_taper_port_b120: face thickness " +
              std::to_string(zMax - zMin) +
              " mm < depth + 2 mm safety margin");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "npt_taper_port_b120 DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ht::NptSpec* s = ht::findNpt(in.thread_size);
    if (!s) throw SkillError("npt_taper_port_b120: thread_size lookup failed");

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError("npt_taper_port_b120: face_id unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const gp_Dir adir = in.axis_dir;
    gp_Pnt entryStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        entryStart = gp_Pnt(in.center_x_mm, in.center_y_mm,
                            (adir.Z() < 0) ? zMax + kOverhang : zMin - kOverhang);
    } else {
        entryStart = gp_Pnt(in.center_x_mm, in.center_y_mm,
                            (zMin + zMax) / 2.0 - adir.Z() * kOverhang);
    }

    const double drillR = s->drill_dia_mm / 2.0;
    const double depth  = s->depth_mm;

    // ── Sub-cut #1: straight pilot drill (tap drill) ─────────────────────
    const gp_Ax2 boreAx(entryStart, adir);
    const TopoDS_Shape pilotBore =
        pr::cylinder(boreAx, drillR, depth + kOverhang);

    // ── Sub-cut #2: conical bore at NPT taper angle ──────────────────────
    // Taper is 1:16 on diameter → over the engagement depth, the diameter
    // shrinks by depth / 16.  Cone is wider at the face (entry).
    const double dTaperDiff = depth * ht::kNptTaperRatio;     // diameter delta
    const double topR    = drillR + dTaperDiff * 0.5 + 0.10;  // slightly oversized at face
    const double botR    = drillR;                            // at engagement bottom
    const gp_Ax2 coneAx(entryStart, adir);
    const TopoDS_Shape taperCone =
        pr::coneFrustum(coneAx, topR, botR, depth + kOverhang);

    TopoDS_Shape unified = pr::fuse(pilotBore, taperCone);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), unified);

    const double volPilot = M_PI * drillR * drillR * depth;
    const double volCone  = M_PI / 3.0 * depth
                            * (topR * topR + topR * botR + botR * botR);
    // True removed ≈ cone (cone fully encloses pilot at most sizes).
    const double volRemoved = std::max(volPilot, volCone);

    json params = {
        { "face_id_resolved", *faceId },
        { "center_x_mm",      in.center_x_mm },
        { "center_y_mm",      in.center_y_mm },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "thread_size",      in.thread_size },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "is_compound",             true },
        { "port_standard",           "NPT_ASME_B1_20_1" },
        { "thread_size",             in.thread_size },
        { "subfeature_count",        2 },
        { "pitch_dia_at_E1_mm",      s->pitch_dia_at_E1_mm },
        { "drill_dia_mm",            s->drill_dia_mm },
        { "port_dia_mm",             s->drill_dia_mm },
        { "depth_mm",                s->depth_mm },
        { "threads_per_inch",        s->threads_per_inch },
        { "taper_per_side_deg",      ht::kNptTaperPerSideDeg },
        { "taper_ratio",             ht::kNptTaperRatio },
        { "derived_volume_removed",  volRemoved },
        { "axis_dir",                { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;npt_tap";
    tooling.tool_dia_mm       = s->drill_dia_mm;
    tooling.tool_material     = "hss";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 100.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.est_cycle_time_s  = std::max(8.0, depth * 0.5 + 4.0);
    tooling.stock_removed_mm3 = volRemoved;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", s->drill_dia_mm },
              { "depth_mm", depth },
              { "role", "straight_pilot" } },
            { { "tool_type", "npt_tap" },
              { "thread", in.thread_size },
              { "taper_per_side_deg", ht::kNptTaperPerSideDeg },
              { "tpi", s->threads_per_inch },
              { "role", "tapered_thread" } },
        } },
        { "standard", "NPT taper per ANSI/ASME B1.20.1 (1:16, ≈1.7899°/side)" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::npt_taper_port_b120 applied: {} drill={} depth={}",
                  in.thread_size, s->drill_dia_mm, depth);

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
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::npt_taper_port_b120
