// @lat: [[engine/skills#helicoil_pilot_compound]]

#include "helicoil_pilot_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>

namespace koocadcam::skill::helicoil_pilot_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

struct HeliEntry {
    const char* size;
    double      sti_tap_drill_mm;
    double      csk_top_dia_mm;
};

constexpr std::array<HeliEntry, 8> kTable {{
    { "M2",   2.10,  2.6  },
    { "M2.5", 2.65,  3.2  },
    { "M3",   3.10,  3.7  },
    { "M4",   4.10,  4.8  },
    { "M5",   5.20,  6.0  },
    { "M6",   6.25,  7.2  },
    { "M8",   8.30,  9.4  },
    { "M10", 10.40, 11.7  },
}};

const HeliEntry* lookupEntry(const std::string& s) {
    for (const auto& e : kTable) if (s == e.size) return &e;
    return nullptr;
}

constexpr double kCskAngleDeg   = 90.0;
constexpr double kClearancePast = 0.5;   // half-D below thread for chips
constexpr double kInsertPilotDepthFactor = 1.5;  // engagement length vs insert
}  // namespace

double stiTapDrillFor(const std::string& s)
{ const auto* e = lookupEntry(s); return e ? e->sti_tap_drill_mm : 0.0; }

double cskTopDiaFor(const std::string& s)
{ const auto* e = lookupEntry(s); return e ? e->csk_top_dia_mm : 0.0; }

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.target_thread_size.empty()) {
        r.add("DFM-HELI-SIZE", "error",
              "helicoil_pilot_compound: target_thread_size is empty");
        return r;
    }
    const auto* e = lookupEntry(in.target_thread_size);
    if (!e) {
        r.add("DFM-HELI-SIZE", "error",
              "helicoil_pilot_compound: unknown target_thread_size '" +
              in.target_thread_size +
              "' (supported: M2/M2.5/M3/M4/M5/M6/M8/M10)");
        return r;
    }

    if (e->sti_tap_drill_mm < 0.8) {
        r.add("DFM-002", "error",
              "helicoil_pilot_compound: tap-drill " +
              std::to_string(e->sti_tap_drill_mm) + " < 0.8 mm");
    }

    if (in.insert_length_mm <= 0.0) {
        r.add("DFM-HELI-DEPTH", "error",
              "helicoil_pilot_compound: insert_length_mm must be > 0");
    }

    // Sanity: insert length recommended ≥ 1×D, ≤ 3×D where D = nominal thread.
    // Strip the leading "M" and parse.
    double nominalD = 0.0;
    try {
        nominalD = std::stod(in.target_thread_size.substr(1));
    } catch (...) { nominalD = 0.0; }
    if (nominalD > 0.0 && in.insert_length_mm > 0.0) {
        const double ratio = in.insert_length_mm / nominalD;
        if (ratio < 0.8) {
            r.add("DFM-HELI-DEPTH", "warning",
                  "helicoil_pilot_compound: insert_length/D ratio " +
                  std::to_string(ratio) + " < 0.8 — under-engaged thread");
        } else if (ratio > 3.5) {
            r.add("DFM-HELI-DEPTH", "warning",
                  "helicoil_pilot_compound: insert_length/D ratio " +
                  std::to_string(ratio) + " > 3.5 — exceeds standard 3×D limit");
        }
    }

    // Stock thickness must accommodate full engagement + clearance.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    const double totalDepth = in.insert_length_mm * kInsertPilotDepthFactor +
                              kClearancePast;
    if (thickness > 0.0 && thickness < totalDepth + 1.0) {
        r.add("DFM-HELI-DEPTH", "error",
              "helicoil_pilot_compound: stock thickness " +
              std::to_string(thickness) + " mm < required pilot depth " +
              std::to_string(totalDepth + 1.0) + " mm");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "helicoil_pilot_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* e = lookupEntry(in.target_thread_size);
    const double tapDrillDia = e->sti_tap_drill_mm;
    const double cskTopDia   = e->csk_top_dia_mm;
    const double threadDepth = in.insert_length_mm * kInsertPilotDepthFactor;
    const double totalDepth  = threadDepth + kClearancePast;
    const double cskAngleRad = kCskAngleDeg * M_PI / 180.0;
    const double cskDepth    = (cskTopDia - tapDrillDia) * 0.5 /
                               std::tan(cskAngleRad / 2.0);

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("helicoil_pilot_compound: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt pilotStart(
        in.position_x_mm - adir.X() * (bboxDiag + kOverhang),
        in.position_y_mm - adir.Y() * (bboxDiag + kOverhang),
        (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kOverhang));
    gp_Pnt entryPlanePoint(pilotStart);

    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        const double entryZ = (adir.Z() < 0) ? zMax : zMin;
        pilotStart      = gp_Pnt(in.position_x_mm, in.position_y_mm,
                                 entryZ - adir.Z() * kOverhang);
        entryPlanePoint = gp_Pnt(in.position_x_mm, in.position_y_mm, entryZ);
    }

    const gp_Ax2 pilotAx (pilotStart,      adir);
    const gp_Ax2 coneAx  (entryPlanePoint, adir);

    // ── Sub-feature 1+2: pilot drill (combined) ──────────────────────────
    // Slice-1 models the tap-thread region as plain cylinder; the chip
    // clearance below extends the same cylinder by kClearancePast.
    // Sub-feature 1 (thread engagement) and sub-feature 2 (chip clearance)
    // share the same diameter — but we model them as TWO sub-features in
    // the signature (the CAM plan uses two ops: drill + tap).
    const TopoDS_Shape pilotTool = pr::cylinder(
        pilotAx, tapDrillDia / 2.0, totalDepth + kOverhang);

    // ── Sub-feature 3: 90° countersink at entry ──────────────────────────
    const TopoDS_Shape coneTool = pr::coneFrustum(
        coneAx, cskTopDia / 2.0, tapDrillDia / 2.0, cskDepth);

    TopoDS_Shape fused = pr::fuse(pilotTool, coneTool);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "entry_face_id",        *entryId },
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
        { "target_thread_size",   in.target_thread_size },
        { "insert_length_mm",     in.insert_length_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "subfeature_count",       3 },   // drill_pilot + thread_pilot + csk
        { "target_thread_size",     in.target_thread_size },
        { "sti_tap_drill_mm",       tapDrillDia },
        { "thread_engagement_mm",   threadDepth },
        { "chip_clearance_mm",      kClearancePast },
        { "csk_top_dia_mm",         cskTopDia },
        { "csk_angle_deg",          kCskAngleDeg },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "drill;sti_tap;countersink";
    tooling.tool_dia_mm = tapDrillDia;
    tooling.tool_length_mm = totalDepth * 1.5 + 5.0;
    tooling.tool_material  = "carbide";
    tooling.flute_count    = 2;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double pilotR = tapDrillDia / 2.0;
    tooling.stock_removed_mm3 = M_PI * pilotR * pilotR * totalDepth;
    tooling.est_cycle_time_s  = std::max(2.0, totalDepth / 40.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },       { "tool_dia_mm", tapDrillDia },
              { "depth_mm",  totalDepth } },
            { { "tool_type", "sti_tap" },     { "thread_size", in.target_thread_size },
              { "engagement_mm", threadDepth } },
            { { "tool_type", "countersink" }, { "tool_dia_mm", cskTopDia },
              { "cone_angle_deg", kCskAngleDeg } },
        } },
        { "insert_standard", "Heli-Coil_MS21209" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::helicoil_pilot_compound applied: {} sti={} len={}",
                  in.target_thread_size, tapDrillDia, in.insert_length_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" },
                                { "is_compound", true } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::helicoil_pilot_compound
