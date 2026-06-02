// @lat: [[engine/skills#cam_follower_threaded_seat]]

#include "cam_follower_threaded_seat.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace koocadcam::skill::cam_follower_threaded_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Cam-follower threaded-seat specs (DIN 5418-style stud follower):
// pilot_dia_mm and nominal_dia_mm come from the central thread table
// (kMetricThreads / findMetric).  nut_od_mm and nut_height_mm are
// cam-follower lock-nut dimensions specific to this skill.

struct CFEntry {
    const char* size;
    double pilot_dia_mm;
    double nut_od_mm;
    double nut_height_mm;
    double nominal_dia_mm;
};

CFEntry makeCFEntry(const char* size, double nut_od, double nut_h)
{
    const auto* m = thread_table::findMetric(size);
    return CFEntry{
        size,
        m ? m->tap_pilot_dia_mm : 0.0,
        nut_od,
        nut_h,
        m ? m->nominal_dia_mm : 0.0,
    };
}

const CFEntry* lookupCF(const std::string& M)
{
    static const std::array<CFEntry, 5> kCFTable {{
        makeCFEntry("M6",  10.0, 5.0 ),
        makeCFEntry("M8",  14.0, 6.5 ),
        makeCFEntry("M10", 18.0, 8.0 ),
        makeCFEntry("M12", 20.0, 10.0),
        makeCFEntry("M16", 26.0, 13.0),
    }};
    for (const auto& e : kCFTable) {
        if (M == e.size) return &e;
    }
    return nullptr;
}

constexpr double kChamferSizeMm = 0.5;

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cam_follower_M.empty()) {
        r.add("DFM-INPUT", "error",
              "cam_follower_threaded_seat: cam_follower_M is empty");
        return r;
    }

    const CFEntry* cf = lookupCF(in.cam_follower_M);
    if (!cf) {
        r.add("DFM-CF-TAP", "error",
              "cam_follower_threaded_seat: unknown cam_follower_M '" +
              in.cam_follower_M + "' (supported: M6/M8/M10/M12/M16)");
        return r;
    }

    if (cf->pilot_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "cam_follower_threaded_seat: pilot dia " +
              std::to_string(cf->pilot_dia_mm) + " mm < 0.8 mm");
    }

    const double tapDepth = (in.tap_depth_mm > 0.0)
        ? in.tap_depth_mm
        : (1.5 * cf->nominal_dia_mm);
    const double minDepth = 1.5 * cf->nominal_dia_mm;
    if (tapDepth < minDepth) {
        r.add("DFM-CF-DEPTH", "error",
              "cam_follower_threaded_seat: tap_depth " +
              std::to_string(tapDepth) +
              " mm < 1.5 × M-dia (" + std::to_string(minDepth) + " mm)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cam_follower_threaded_seat DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("cam_follower_threaded_seat: entry_face datum unresolved");

    const CFEntry* cf = lookupCF(in.cam_follower_M);
    if (!cf) throw SkillError("cam_follower_threaded_seat: unknown M (post-validate)");

    const double tapDepth = (in.tap_depth_mm > 0.0)
        ? in.tap_depth_mm
        : (1.5 * cf->nominal_dia_mm);
    const double cboreDia    = cf->nut_od_mm + 1.0;   // lock-nut OD + 1 mm clearance
    const double cboreDepth  = cf->nut_height_mm;
    const double pilotDia    = cf->pilot_dia_mm;
    const double chamferOD   = pilotDia + 2.0 * kChamferSizeMm;  // entry chamfer outer dia

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax-xMin)*(xMax-xMin) + (yMax-yMin)*(yMax-yMin) + (zMax-zMin)*(zMax-zMin));

    constexpr double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kOverhang);
        }
    } else {
        const double margin = bboxDiag + 1.0;
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * margin,
            in.position_y_mm - adir.Y() * margin,
            (zMin + zMax)/2.0 - adir.Z() * margin);
    }

    const gp_Ax2 toolAx(toolStart, adir);

    // ── Sub-feature 1: tap pilot (smallest dia, deepest) ──────────────
    const TopoDS_Shape pilotTool =
        pr::cylinder(toolAx, pilotDia / 2.0,
                     cboreDepth + tapDepth + kOverhang);

    // ── Sub-feature 2: counterbore for lock nut (larger dia, shallow) ─
    const TopoDS_Shape cboreTool =
        pr::cylinder(toolAx, cboreDia / 2.0, cboreDepth + kOverhang);

    // ── Sub-feature 3: entry chamfer ───────────────────────────────────
    // Modelled as a tiny outer cylindrical removal at the very entry —
    // approximates a 0.5 mm chamfer by widening the top by kChamferSizeMm.
    const TopoDS_Shape chamferTool =
        pr::cylinder(toolAx, chamferOD / 2.0, kChamferSizeMm + kOverhang);

    // All three are concentric and overlapping → fuse then single cut.
    const TopoDS_Shape fusedCutter =
        pr::fuseMany(pilotTool, { cboreTool, chamferTool });

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedCutter);

    constexpr int kSubFeatures = 3;

    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "cam_follower_M",  in.cam_follower_M },
        { "tap_depth_mm",    in.tap_depth_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    kSubFeatures },
        { "tap_pilot_count",     1 },
        { "counterbore_count",   1 },
        { "chamfer_count",       1 },
        { "cam_follower_M",      in.cam_follower_M },
        { "pilot_dia_mm",        pilotDia },
        { "tap_depth_mm",        tapDepth },
        { "counterbore_dia_mm",  cboreDia },
        { "counterbore_depth_mm",cboreDepth },
        { "chamfer_size_mm",     kChamferSizeMm },
        // Total feature depth = chamfer + cbore + tap
        { "total_depth_mm",      kChamferSizeMm + cboreDepth + tapDepth },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "drill;counterbore;tap;chamfer_mill";
    tooling.tool_dia_mm      = cboreDia;
    tooling.tool_length_mm   = (cboreDepth + tapDepth) * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double pilotVol    = M_PI * (pilotDia/2.0)*(pilotDia/2.0) * (cboreDepth + tapDepth);
    const double cboreVol    = M_PI * ((cboreDia/2.0)*(cboreDia/2.0)
                              - (pilotDia/2.0)*(pilotDia/2.0)) * cboreDepth;
    const double chamferVol  = M_PI * ((chamferOD/2.0)*(chamferOD/2.0)
                              - (cboreDia/2.0)*(cboreDia/2.0)) * kChamferSizeMm;
    tooling.stock_removed_mm3 = pilotVol + cboreVol + chamferVol;
    tooling.est_cycle_time_s  = std::max(3.0,
        (tapDepth + cboreDepth) * 0.3 + 2.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "chamfer_mill" },
              { "purpose",   "entry chamfer" },
              { "chamfer_size_mm", kChamferSizeMm } },
            { { "tool_type", "counterbore" },
              { "purpose",   "lock-nut seat" },
              { "tool_dia_mm", cboreDia },
              { "depth_mm",    cboreDepth } },
            { { "tool_type", "drill" },
              { "purpose",   "tap pilot" },
              { "tool_dia_mm", pilotDia },
              { "depth_mm",    tapDepth + cboreDepth } },
            { { "tool_type", "tap" },
              { "thread_size", in.cam_follower_M },
              { "tap_depth_mm", tapDepth } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::cam_follower_threaded_seat applied: M={} tap_d={} cbore_d={}",
                  in.cam_follower_M, tapDepth, cboreDepth);

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
        r.matched_geometry = {
            { "source",            "metadata_replay" },
            { "subfeature_count",  f.pattern.value("subfeature_count", 0) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::cam_follower_threaded_seat
