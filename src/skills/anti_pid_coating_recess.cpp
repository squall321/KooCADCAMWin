// @lat: [[engine/skills#anti_pid_coating_recess]]

#include "anti_pid_coating_recess.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace koocadcam::skill::anti_pid_coating_recess {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "anti_pid_coating_recess: face_id out of range");
    }
    if (in.cell_array_count_x <= 0 || in.cell_array_count_y <= 0 ||
        !(in.cell_pitch_x_mm > 0.0) || !(in.cell_pitch_y_mm > 0.0) ||
        !(in.recess_depth_mm > 0.0) || !(in.coating_margin_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "anti_pid_coating_recess: all counts and dimensions must be > 0");
        return r;
    }

    if (in.cell_array_count_x > 24 || in.cell_array_count_y > 24) {
        r.add("DFM-PID-ARRAY-SIZE", "error",
              "anti_pid_coating_recess: cell array > 24 in either axis "
              "(got " + std::to_string(in.cell_array_count_x) + " × " +
              std::to_string(in.cell_array_count_y) + ")");
    }

    if (in.recess_depth_mm < 0.1 || in.recess_depth_mm > 1.5) {
        r.add("DFM-PID-DEPTH", "error",
              "anti_pid_coating_recess: recess_depth " +
              std::to_string(in.recess_depth_mm) +
              " mm outside anti-PID coating range [0.1, 1.5] mm");
    }

    if (in.cell_pitch_x_mm < 2.0 || in.cell_pitch_y_mm < 2.0) {
        r.add("DFM-PID-PITCH", "error",
              "anti_pid_coating_recess: cell pitch must be >= 2 mm");
    }

    const double minPitch = std::min(in.cell_pitch_x_mm, in.cell_pitch_y_mm);
    if (in.coating_margin_mm >= minPitch / 2.0) {
        r.add("DFM-PID-MARGIN", "error",
              "anti_pid_coating_recess: coating_margin (" +
              std::to_string(in.coating_margin_mm) +
              ") must be < min(cell_pitch)/2 = " +
              std::to_string(minPitch / 2.0));
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "anti_pid_coating_recess DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const double baseZ = faceC.Z();
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());

    constexpr double kEmbed = 0.05;
    const double recessW = in.cell_pitch_x_mm - 2.0 * in.coating_margin_mm;
    const double recessH = in.cell_pitch_y_mm - 2.0 * in.coating_margin_mm;
    const double recessD = in.recess_depth_mm + kEmbed;

    // ── SEQUENTIAL pr::cut for each cell recess ──────────────────────────
    TopoDS_Shape current = wp.shape();
    double totalRemoved = 0.0;

    for (int iy = 0; iy < in.cell_array_count_y; ++iy) {
        for (int ix = 0; ix < in.cell_array_count_x; ++ix) {
            const double cx = in.origin_x_mm + ix * in.cell_pitch_x_mm;
            const double cy = in.origin_y_mm + iy * in.cell_pitch_y_mm;

            const gp_Pnt recessOrigin(
                cx - recessW / 2.0,
                cy - recessH / 2.0,
                baseZ + kEmbed);
            const gp_Ax2 recessAx(recessOrigin, drillDir);
            const TopoDS_Shape recess = pr::box(
                recessAx, recessW, recessH, recessD);
            current = pr::cut(current, recess);
            totalRemoved += recessW * recessH * in.recess_depth_mm;
        }
    }

    const TopoDS_Shape newShape = current;
    const int cellCount = in.cell_array_count_x * in.cell_array_count_y;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",            in.face_id },
        { "origin_x_mm",        in.origin_x_mm },
        { "origin_y_mm",        in.origin_y_mm },
        { "cell_array_count_x", in.cell_array_count_x },
        { "cell_array_count_y", in.cell_array_count_y },
        { "cell_pitch_x_mm",    in.cell_pitch_x_mm },
        { "cell_pitch_y_mm",    in.cell_pitch_y_mm },
        { "recess_depth_mm",    in.recess_depth_mm },
        { "coating_margin_mm",  in.coating_margin_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_solar_feature",    true },
        { "solar_feature_type",  "anti_pid_recess_array" },
        { "subfeature_count",    cellCount },
        { "cell_count",          cellCount },
        { "recess_depth_mm",     in.recess_depth_mm },
        { "coating_margin_mm",   in.coating_margin_mm },
        { "derived_volume_removed_mm3", totalRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = std::min(recessW, recessH) * 0.4;
    tooling.tool_length_mm    = in.recess_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 500.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = totalRemoved;
    tooling.est_cycle_time_s  = std::max(5.0, totalRemoved / 80.0);
    tooling.extra = {
        { "feature_standard", "Anti-PID coating recess (IEC 62804)" },
        { "cell_count",       cellCount },
        { "removed_volume_mm3", totalRemoved },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::anti_pid_coating_recess applied: nx={} ny={} depth={}",
        in.cell_array_count_x, in.cell_array_count_y, in.recess_depth_mm);

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
            { "solar_feature_type", "anti_pid_recess_array" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::anti_pid_coating_recess
