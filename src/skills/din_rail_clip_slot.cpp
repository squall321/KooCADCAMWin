// @lat: [[engine/skills#din_rail_clip_slot]]

#include "din_rail_clip_slot.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::din_rail_clip_slot {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── EN 60715 TS35 hat-rail dimensions (mm) ──────────────────────────────
namespace {
constexpr double kRailGrooveWidth_mm  = 27.5;   // main groove width  (cut #1)
constexpr double kRailGrooveDepth_mm  =  5.5;   // main groove depth
constexpr double kHookOffset_mm       = 17.0;   // hook from rail centerline
constexpr double kHookWidth_mm        =  5.0;   // hook ledge width   (cut #2)
constexpr double kHookDepth_mm        =  2.0;   // hook ledge depth
constexpr double kChamferBreak_mm     =  0.8;   // upper edge chamfer (cut #3)
constexpr double kMinFaceThickness_mm =  8.0;   // groove + back-wall budget
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.rail_length_mm < 40.0) {
        r.add("DFM-INPUT", "error",
              "din_rail_clip_slot: rail_length_mm " +
              std::to_string(in.rail_length_mm) + " < 40 mm (rail clip min)");
    }
    if (in.rail_length_mm > 500.0) {
        r.add("DFM-INPUT", "error",
              "din_rail_clip_slot: rail_length_mm " +
              std::to_string(in.rail_length_mm) + " > 500 mm (single-pass limit)");
    }

    // Panel thickness check: groove takes 5.5 mm + chamfer ≈ 0.8 mm + need
    // ≥ 1.5 mm back-wall = 7.8 mm.  Round to 8 mm minimum.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness < kMinFaceThickness_mm) {
        r.add("DFM-DIN-WALL", "error",
              "din_rail_clip_slot: enclosure face thickness " +
              std::to_string(thickness) + " mm < min " +
              std::to_string(kMinFaceThickness_mm) + " mm (groove + back-wall)");
    }

    // Geometry sanity: the chamfer cannot be sub-mill (DFM-002).
    if constexpr (kChamferBreak_mm < 0.5) {
        r.add("DFM-002", "error",
              "din_rail_clip_slot: chamfer break < 0.5 mm");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "din_rail_clip_slot DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.enclosure_face);
    if (!entryId) throw SkillError("din_rail_clip_slot: enclosure_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // Rail axis frame: rotation `rail_dir_deg` around face normal (axis_dir).
    const double rad = in.rail_dir_deg * M_PI / 180.0;
    const gp_Dir longDir(std::cos(rad), std::sin(rad), 0.0);
    const gp_Dir crossDir(-std::sin(rad), std::cos(rad), 0.0);

    // We always cut from the +Z (top) face.  All three cutters share the
    // same downward axis_dir = -Z and a small overhang for clean Booleans.
    constexpr double kOverhang = 0.05;
    const double topZ = zMax;

    // ── Cut #1 : top rail groove (27.5 × rail_length × 5.5 deep) ────────
    //
    // Box origin centred on (center_x, center_y), then offset by half-width
    // along the crossDir and half-length along longDir.  gp_Ax2(P, +Z, longDir)
    // → DX=length, DY=width, DZ=depth.
    const gp_Pnt grooveOrigin(
        in.center_x_mm - (in.rail_length_mm / 2.0) * longDir.X()
                       - (kRailGrooveWidth_mm / 2.0) * crossDir.X(),
        in.center_y_mm - (in.rail_length_mm / 2.0) * longDir.Y()
                       - (kRailGrooveWidth_mm / 2.0) * crossDir.Y(),
        topZ - kRailGrooveDepth_mm);
    const gp_Ax2 grooveAx(grooveOrigin, gp_Dir(0.0, 0.0, 1.0), longDir);
    const TopoDS_Shape grooveCutter = pr::box(grooveAx, in.rail_length_mm,
                                              kRailGrooveWidth_mm,
                                              kRailGrooveDepth_mm + kOverhang);

    // ── Cut #2 : hook ledge (5 × rail_length × 2 deep), offset by 17 mm ─
    const gp_Pnt hookOrigin(
        in.center_x_mm - (in.rail_length_mm / 2.0) * longDir.X()
                       + (kHookOffset_mm - kHookWidth_mm / 2.0) * crossDir.X(),
        in.center_y_mm - (in.rail_length_mm / 2.0) * longDir.Y()
                       + (kHookOffset_mm - kHookWidth_mm / 2.0) * crossDir.Y(),
        topZ - kHookDepth_mm);
    const gp_Ax2 hookAx(hookOrigin, gp_Dir(0.0, 0.0, 1.0), longDir);
    const TopoDS_Shape hookCutter = pr::box(hookAx, in.rail_length_mm,
                                            kHookWidth_mm,
                                            kHookDepth_mm + kOverhang);

    // ── Cut #3 : retention chamfer (0.8 × 45° break) ────────────────────
    //
    // Modelled as a thin sloped slab along the long edge of the groove.
    // For slice-1 simplicity we approximate the chamfer as a 0.8 × 0.8 mm
    // axis-aligned ledge that intersects the corner — this is a single
    // additional cut visible on the recognise side as the small upper
    // shoulder.
    const gp_Pnt chamferOrigin(
        in.center_x_mm - (in.rail_length_mm / 2.0) * longDir.X()
                       - (kRailGrooveWidth_mm / 2.0 + kChamferBreak_mm) * crossDir.X(),
        in.center_y_mm - (in.rail_length_mm / 2.0) * longDir.Y()
                       - (kRailGrooveWidth_mm / 2.0 + kChamferBreak_mm) * crossDir.Y(),
        topZ - kChamferBreak_mm);
    const gp_Ax2 chamferAx(chamferOrigin, gp_Dir(0.0, 0.0, 1.0), longDir);
    const TopoDS_Shape chamferCutter = pr::box(chamferAx, in.rail_length_mm,
                                               kChamferBreak_mm,
                                               kChamferBreak_mm + kOverhang);

    // ── Boolean: cuts #1, #2, #3 are NOT fully overlapping; combine with
    //    cutMany so we get one efficient cut.  The chamfer overlaps the
    //    groove top edge — the union resolves naturally.
    const TopoDS_Shape newShape = pr::cutMany(wp.shape(),
        { grooveCutter, hookCutter, chamferCutter });

    // ── Signature ───────────────────────────────────────────────────────
    json params = {
        { "entry_face_id",  *entryId },
        { "center_x_mm",    in.center_x_mm },
        { "center_y_mm",    in.center_y_mm },
        { "axis_dir",       { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "rail_dir_deg",   in.rail_dir_deg },
        { "rail_length_mm", in.rail_length_mm },
    };

    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    3 },
        { "rail_length_mm",      in.rail_length_mm },
        { "rail_dir_deg",        in.rail_dir_deg },
        { "groove_width_mm",     kRailGrooveWidth_mm },
        { "groove_depth_mm",     kRailGrooveDepth_mm },
        { "hook_offset_mm",      kHookOffset_mm },
        { "hook_width_mm",       kHookWidth_mm },
        { "hook_depth_mm",       kHookDepth_mm },
        { "chamfer_break_mm",    kChamferBreak_mm },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "subfeatures",         {
            { { "kind", "rail_groove" },
              { "size_mm", { in.rail_length_mm, kRailGrooveWidth_mm, kRailGrooveDepth_mm } } },
            { { "kind", "hook_ledge" },
              { "size_mm", { in.rail_length_mm, kHookWidth_mm, kHookDepth_mm } },
              { "offset_mm", kHookOffset_mm } },
            { { "kind", "retention_chamfer" },
              { "size_mm", { in.rail_length_mm, kChamferBreak_mm, kChamferBreak_mm } } },
        } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "end_mill;t_slot_cutter";
    tooling.tool_dia_mm      = kRailGrooveWidth_mm;
    tooling.tool_length_mm   = kRailGrooveDepth_mm * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 4;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double grooveVol = in.rail_length_mm * kRailGrooveWidth_mm * kRailGrooveDepth_mm;
    const double hookVol   = in.rail_length_mm * kHookWidth_mm        * kHookDepth_mm;
    const double chamVol   = in.rail_length_mm * kChamferBreak_mm     * kChamferBreak_mm;
    tooling.stock_removed_mm3 = grooveVol + hookVol + chamVol;
    tooling.est_cycle_time_s  = std::max(3.0, in.rail_length_mm / 30.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "end_mill" },     { "tool_dia_mm", 6.0 },
              { "note", "rail groove primary cut" } },
            { { "tool_type", "end_mill" },     { "tool_dia_mm", 3.0 },
              { "note", "hook ledge undercut" } },
            { { "tool_type", "chamfer_mill" }, { "tool_dia_mm", 6.0 },
              { "note", "0.8 mm × 45° break on rail edge" } },
        } },
        { "din_standard", "EN 60715 TS35" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::din_rail_clip_slot applied: rail_length={} faces {}→{}",
                  in.rail_length_mm, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Compound metadata replay: a DIN-rail clip slot leaves a fairly distinctive
// geometric signature (one wide shallow groove + a parallel narrow hook
// groove + a chamfer break) but reliably re-extracting all three primitives
// from a B-Rep post-STEP-import is fragile.  We follow the slice-1 convention
// for compound macros: scan the workpiece's feature history for a stamped
// din_rail_clip_slot signature.  When present, confidence = 1.0 (we trust
// the in-process metadata).  When absent, recognize() returns empty.
std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        const auto& p = f.pattern;
        json recovered = {
            { "center_x_mm",    f.params.value("center_x_mm",    0.0) },
            { "center_y_mm",    f.params.value("center_y_mm",    0.0) },
            { "rail_dir_deg",   f.params.value("rail_dir_deg",   0.0) },
            { "rail_length_mm", f.params.value("rail_length_mm", 0.0) },
            { "axis_dir",       f.params.value("axis_dir",       json::array({0.0,0.0,-1.0})) },
        };
        json matched = {
            { "source",            "feature_history_replay" },
            { "subfeature_count",  p.value("subfeature_count", 0) },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 1.0, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::din_rail_clip_slot
