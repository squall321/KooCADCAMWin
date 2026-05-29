// @lat: [[engine/skills#vibration_isolator_seat]]

#include "vibration_isolator_seat.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <Standard_Failure.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::vibration_isolator_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Stud size lookup table ───────────────────────────────────────────────
//
// Clearance pilot, socket-nut head dia (across-flats × 1.155), and a few
// shoulder relief defaults.  ISO 4762 + SAE J429 values for common sizes.

namespace {

struct StudSpec
{
    const char* size;
    double pilot_dia_mm;   // tap-drill / clearance pilot (close fit)
    double nut_head_dia_mm; // socket nut head across-corners
    double nut_height_mm;   // standard nut thickness
};

constexpr std::array<StudSpec, 6> kStudTable {{
    { "M4",  4.5,  7.7,  3.2 },
    { "M5",  5.5,  8.8,  4.0 },
    { "M6",  6.6, 11.0,  5.0 },
    { "M8",  9.0, 14.4,  6.5 },
    { "M10", 11.0, 17.8, 8.0 },
    { "M12", 14.0, 20.0, 10.0 },
}};

const StudSpec* findStud(const std::string& size)
{
    for (const auto& s : kStudTable) {
        if (size == s.size) return &s;
    }
    return nullptr;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const StudSpec* stud = findStud(in.stud_M);
    if (!stud) {
        r.add("DFM-STUD", "error",
              "vibration_isolator_seat: unknown stud_M '" + in.stud_M +
              "' (supported: M4/M5/M6/M8/M10/M12)");
        return r;
    }

    if (stud->pilot_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "vibration_isolator_seat pilot_dia " +
              std::to_string(stud->pilot_dia_mm) + " mm < min 0.8 mm");
    }

    if (in.bushing_od_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "vibration_isolator_seat: bushing_od_mm must be > 0");
    } else if (in.bushing_od_mm < stud->pilot_dia_mm + 2.0) {
        r.add("DFM-BUSHING", "error",
              "vibration_isolator_seat: bushing_od " +
              std::to_string(in.bushing_od_mm) +
              " mm < pilot_dia + 2 mm — no annular shoulder for bushing seat");
    }

    // Lockwasher relief outer dia = cb_dia + 1.5 mm.  Need bushing seat
    // outer > lockwasher relief outer so the seat sits OUTSIDE the relief.
    const double cb_dia    = stud->nut_head_dia_mm;
    const double relief_od = cb_dia + 1.5;
    if (in.bushing_od_mm <= relief_od + 0.5) {
        r.add("DFM-BUSHING", "warning",
              "vibration_isolator_seat: bushing_od (" +
              std::to_string(in.bushing_od_mm) +
              " mm) ≤ lockwasher_relief_od (" +
              std::to_string(relief_od) +
              " mm) + 0.5 mm — recommend wider bushing or smaller stud");
    }

    // Total seat depth: bushing_seat_depth + cb_depth + lockwasher_relief
    // depth.  Must be < plate_thick.
    const double bushing_seat_depth = in.bushing_od_mm * 0.4;
    const double cb_depth           = stud->nut_height_mm + 1.0;  // +1 mm clearance
    const double relief_depth       = 0.6;
    const double total_seat_depth   = bushing_seat_depth + relief_depth + cb_depth;

    if (in.plate_thick_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "vibration_isolator_seat: plate_thick_mm must be > 0");
    } else if (in.plate_thick_mm < total_seat_depth + 1.0) {
        r.add("DFM-PLATE-T", "error",
              "vibration_isolator_seat: plate_thick " +
              std::to_string(in.plate_thick_mm) +
              " mm < total_seat_depth (" +
              std::to_string(total_seat_depth) +
              " mm) + 1 mm — seat would cut through plate");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "vibration_isolator_seat DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const StudSpec* stud = findStud(in.stud_M);
    if (!stud) throw SkillError("vibration_isolator_seat: stud spec lost after validation");

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("vibration_isolator_seat: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("vibration_isolator_seat: only axis_dir = -Z supported in slice-1");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    // ── Sub-feature derived dimensions ─────────────────────────────────
    const double pilot_dia    = stud->pilot_dia_mm;
    const double cb_dia       = stud->nut_head_dia_mm;
    const double cb_depth     = stud->nut_height_mm + 1.0;
    const double relief_od    = cb_dia + 1.5;
    const double relief_id    = cb_dia;
    const double relief_depth = 0.6;
    const double bushing_seat_depth = in.bushing_od_mm * 0.4;

    // Cylinder layout — all concentric on axis_dir, around (px, py).
    // Top of stock = zMax.
    //   Bushing seat: from zMax  → zMax - bushing_seat_depth
    //   Lockwasher relief: from (zMax - bushing_seat_depth) → that - relief_depth
    //   Counterbore (for nut head): from (zMax - bushing_seat_depth - relief_depth)
    //                               → that - cb_depth
    //   Pilot through: from zMax → zMin - kOverhang  (entire plate)

    const gp_Pnt axisTop(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
    const gp_Ax2 toolAx(axisTop, in.axis_dir);

    // 1) Pilot through hole.
    const double pilotH = (zMax - zMin) + 2.0 * kOverhang;
    const TopoDS_Shape pilotTool = pr::cylinder(toolAx, pilot_dia / 2.0, pilotH);

    // 2) Bushing seat (top recess).
    const TopoDS_Shape bushingTool = pr::cylinder(toolAx, in.bushing_od_mm / 2.0,
                                                  bushing_seat_depth + kOverhang);

    // 3) Counterbore for nut head.  Starts AFTER bushing seat + relief depth.
    const gp_Pnt cbTop(
        in.position_x_mm, in.position_y_mm,
        zMax - bushing_seat_depth - relief_depth + kOverhang);
    const gp_Ax2 cbAx(cbTop, in.axis_dir);
    const TopoDS_Shape cbTool = pr::cylinder(cbAx, cb_dia / 2.0,
                                             cb_depth + kOverhang);

    // 4) Lockwasher relief annulus — at intermediate depth.
    const gp_Pnt reliefTop(
        in.position_x_mm, in.position_y_mm,
        zMax - bushing_seat_depth + kOverhang / 2.0);
    const gp_Ax2 reliefAx(reliefTop, in.axis_dir);
    // Annular ring tool.
    const TopoDS_Shape reliefTool = pr::annularRing(
        reliefAx, relief_od / 2.0, relief_id / 2.0, relief_depth + kOverhang);

    // Fuse all 4 concentric tools into a single cutter (concentric
    // overlap → fuse_first_then_cut from counterbore.cpp).
    TopoDS_Shape fused;
    try {
        fused = pr::fuse(pilotTool, bushingTool);
        fused = pr::fuse(fused, cbTool);
        fused = pr::fuse(fused, reliefTool);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("vibration_isolator_seat: fuse failed: ") + e.what());
    }

    TopoDS_Shape newShape;
    try {
        newShape = pr::cut(wp.shape(), fused);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("vibration_isolator_seat: cut failed: ") + e.what());
    }

    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "stud_M",          in.stud_M },
        { "bushing_od_mm",   in.bushing_od_mm },
        { "plate_thick_mm",  in.plate_thick_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    4 },
        { "stud_M",              in.stud_M },
        { "bushing_od_mm",       in.bushing_od_mm },
        { "plate_thick_mm",      in.plate_thick_mm },
        { "pilot_dia_mm",        pilot_dia },
        { "cb_dia_mm",           cb_dia },
        { "cb_depth_mm",         cb_depth },
        { "relief_od_mm",        relief_od },
        { "relief_id_mm",        relief_id },
        { "relief_depth_mm",     relief_depth },
        { "bushing_seat_depth_mm", bushing_seat_depth },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type   = "drill;cbore;endmill;groove";
    tooling.tool_dia_mm = in.bushing_od_mm;
    tooling.tool_material = "carbide";
    tooling.flute_count   = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    // Total stock removed (approx sum of 4 sub-volumes).
    const double pilotV   = M_PI * (pilot_dia/2.0) * (pilot_dia/2.0) * in.plate_thick_mm;
    const double bushingV = M_PI * (in.bushing_od_mm/2.0) * (in.bushing_od_mm/2.0)
                            * bushing_seat_depth - pilotV * (bushing_seat_depth / in.plate_thick_mm);
    const double cbV = M_PI * (cb_dia/2.0) * (cb_dia/2.0) * cb_depth
                       - pilotV * (cb_depth / in.plate_thick_mm);
    const double reliefV = M_PI * ((relief_od/2.0)*(relief_od/2.0)
                                   - (relief_id/2.0)*(relief_id/2.0)) * relief_depth;
    tooling.stock_removed_mm3 = std::max(0.0, pilotV + bushingV + cbV + reliefV);
    tooling.est_cycle_time_s = std::max(5.0, in.plate_thick_mm / 30.0 + 4.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },     { "tool_dia_mm", pilot_dia },
              { "depth_mm", in.plate_thick_mm + kOverhang } },
            { { "tool_type", "endmill" },   { "tool_dia_mm", in.bushing_od_mm },
              { "depth_mm", bushing_seat_depth } },
            { { "tool_type", "groove" },    { "tool_od_mm", relief_od },
              { "tool_id_mm", relief_id }, { "depth_mm", relief_depth } },
            { { "tool_type", "cbore" },     { "tool_dia_mm", cb_dia },
              { "depth_mm", cb_depth } },
        } },
        { "standard", "SAE J429 / ISO 4762 nut head; bushing per Lord/Trelleborg" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::vibration_isolator_seat applied: stud={} bushing_od={} plate={} faces {}→{}",
                  in.stud_M, in.bushing_od_mm, in.plate_thick_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Compound concentric-cylinder + annular-groove signatures are not unique
// from B-rep alone — could overlap with counterbore + pocket combos.
// Slice-1 strategy: metadata replay only.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "feature_history_replay" } };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::vibration_isolator_seat
