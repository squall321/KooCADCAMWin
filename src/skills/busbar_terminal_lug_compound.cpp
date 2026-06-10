// @lat: [[engine/skills#busbar_terminal_lug_compound]]

#include "busbar_terminal_lug_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::busbar_terminal_lug_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
bool isStandardTerminalDia(double d)
{
    constexpr double allowed[] = { 8.0, 10.0, 12.0 };
    for (double a : allowed) if (std::abs(a - d) < 0.05) return true;
    return false;
}
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "busbar_terminal_lug_compound: face_id out of range");
    }
    if (!(in.busbar_width_mm > 0.0) || !(in.busbar_thickness_mm > 0.0) ||
        !(in.terminal_dia_mm > 0.0) || !(in.terminal_pcd_mm > 0.0) ||
        in.lug_terminal_count <= 0) {
        r.add("DFM-INPUT", "error",
              "busbar_terminal_lug_compound: all dimensions and counts > 0");
        return r;
    }

    if (!isStandardTerminalDia(in.terminal_dia_mm)) {
        r.add("DFM-LUG-TERMINAL-DIA", "error",
              "busbar_terminal_lug_compound: terminal_dia " +
              std::to_string(in.terminal_dia_mm) +
              " mm not in standard PV set {M8, M10, M12}");
    }

    if (in.lug_terminal_count != 1 && in.lug_terminal_count != 2 &&
        in.lug_terminal_count != 4) {
        r.add("DFM-LUG-COUNT", "error",
              "busbar_terminal_lug_compound: lug_terminal_count must be 1, 2 or 4");
    }

    if (in.busbar_thickness_mm < 3.0) {
        r.add("DFM-IEEE605", "error",
              "busbar_terminal_lug_compound: busbar_thickness " +
              std::to_string(in.busbar_thickness_mm) +
              " mm < 3 mm minimum current-carrying cross-section");
    }
    if (in.busbar_thickness_mm > 12.0) {
        r.add("DFM-LUG-THICK", "warning",
              "busbar_terminal_lug_compound: busbar_thickness > 12 mm "
              "is unusual for PV combiner-box bars");
    }

    // PCD + terminal dia must fit inside the busbar width.
    if (in.busbar_width_mm < in.terminal_pcd_mm + 2.0 * in.terminal_dia_mm) {
        r.add("DFM-LUG-PCD-FIT", "error",
              "busbar_terminal_lug_compound: busbar_width must be >= "
              "terminal_pcd + 2 × terminal_dia (edge distance)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "busbar_terminal_lug_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const double baseZ = faceC.Z();
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());

    constexpr double kEmbed = 0.05;
    const double termR = in.terminal_dia_mm / 2.0;
    const double pcdR  = in.terminal_pcd_mm / 2.0;

    // Busbar slot length runs along +X (lug centerline), width along +Y.
    // Slot length = terminal_pcd + 2 × terminal_dia (enough to clear all
    // terminals when arranged along ±X).
    const double slotLength = in.terminal_pcd_mm + 2.0 * in.terminal_dia_mm;
    const double slotWidth  = in.busbar_width_mm;

    // ── 1. CUT busbar slot at face ───────────────────────────────────────
    const gp_Pnt slotOrigin(
        in.center_x_mm - slotLength / 2.0,
        in.center_y_mm - slotWidth  / 2.0,
        baseZ + kEmbed);
    const gp_Ax2 slotAx(slotOrigin, drillDir);
    const TopoDS_Shape slotTool = pr::box(
        slotAx, slotLength, slotWidth,
        in.busbar_thickness_mm + kEmbed);
    TopoDS_Shape current = pr::cut(wp.shape(), slotTool);

    // ── 2..N+1. CUT N terminal bolt holes around lug centerline ──────────
    // Terminals positioned on circle of radius pcdR centred on (cx, cy).
    // For count = 1: place on +X.
    // For count = 2: place on ±X.
    // For count = 4: place on ±X and ±Y.
    const double drillThru = in.busbar_thickness_mm + 5.0 + 2.0 * kEmbed;

    double totalHoleVol = 0.0;
    for (int k = 0; k < in.lug_terminal_count; ++k) {
        double angle = 0.0;
        if (in.lug_terminal_count == 1) angle = 0.0;
        else if (in.lug_terminal_count == 2) angle = (k == 0) ? 0.0 : M_PI;
        else /* 4 */ angle = (M_PI / 2.0) * k;
        const double tx = in.center_x_mm + pcdR * std::cos(angle);
        const double ty = in.center_y_mm + pcdR * std::sin(angle);
        const gp_Pnt tOrigin(tx, ty, baseZ + kEmbed);
        const gp_Ax2 tAx(tOrigin, drillDir);
        const TopoDS_Shape termHole = pr::cylinder(tAx, termR, drillThru);
        current = pr::cut(current, termHole);
        totalHoleVol += M_PI * termR * termR * drillThru;
    }

    const TopoDS_Shape newShape = current;

    // ── Volumes ──────────────────────────────────────────────────────────
    const double slotVol = slotLength * slotWidth * in.busbar_thickness_mm;
    const double removedVol = slotVol + totalHoleVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",             in.face_id },
        { "center_x_mm",         in.center_x_mm },
        { "center_y_mm",         in.center_y_mm },
        { "busbar_width_mm",     in.busbar_width_mm },
        { "busbar_thickness_mm", in.busbar_thickness_mm },
        { "lug_terminal_count",  in.lug_terminal_count },
        { "terminal_dia_mm",     in.terminal_dia_mm },
        { "terminal_pcd_mm",     in.terminal_pcd_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_solar_feature",    true },
        { "solar_feature_type",  "busbar_terminal_lug" },
        { "subfeature_count",    1 + in.lug_terminal_count },
        { "terminal_count",      in.lug_terminal_count },
        { "terminal_dia_mm",     in.terminal_dia_mm },
        { "terminal_pcd_mm",     in.terminal_pcd_mm },
        { "derived_volume_removed_mm3", removedVol },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill;tap";
    tooling.tool_dia_mm       = in.terminal_dia_mm;
    tooling.tool_length_mm    = drillThru + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(6.0, removedVol / 250.0);
    tooling.extra = {
        { "feature_standard", "PV combiner-box busbar lug (IEC 61439)" },
        { "removed_volume_mm3", removedVol },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::busbar_terminal_lug_compound applied: count={} termD={} pcd={}",
        in.lug_terminal_count, in.terminal_dia_mm, in.terminal_pcd_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay ─────────────────────────────────────────

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
            { "source",             "metadata_replay" },
            { "is_compound",        true },
            { "solar_feature_type", "busbar_terminal_lug" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::busbar_terminal_lug_compound
