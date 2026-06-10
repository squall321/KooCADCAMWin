// @lat: [[engine/skills#heatsink_blade_fin_extruded]]

#include "heatsink_blade_fin_extruded.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace koocadcam::skill::heatsink_blade_fin_extruded {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.fin_count < 2) {
        r.add("DFM-FIN-N", "error",
              "heatsink_blade_fin_extruded: fin_count must be >= 2");
    }
    if (!(in.fin_thickness_mm >= 0.5)) {
        r.add("DFM-FIN-THK", "error",
              "heatsink_blade_fin_extruded: fin_thickness_mm " +
              std::to_string(in.fin_thickness_mm) +
              " < 0.5 mm (cutter / sheet floor)");
    }
    if (!(in.fin_height_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "heatsink_blade_fin_extruded: fin_height_mm must be > 0");
    }
    if (!(in.fin_length_mm > 0.0)) {
        r.add("DFM-FIN-LEN", "error",
              "heatsink_blade_fin_extruded: fin_length_mm must be > 0");
    }
    if (in.fin_thickness_mm > 0.0 && in.fin_height_mm > 0.0) {
        const double ar = in.fin_height_mm / in.fin_thickness_mm;
        if (ar > 12.0) {
            r.add("DFM-FIN-AR", "error",
                  "heatsink_blade_fin_extruded: AR " + std::to_string(ar) +
                  " > 12 : 1 (cutter deflection / extrusion limit)");
        }
    }
    if (in.fin_thickness_mm > 0.0 && in.fin_pitch_mm > 0.0 &&
        in.fin_pitch_mm <= in.fin_thickness_mm) {
        r.add("DFM-FIN-PITCH", "error",
              "heatsink_blade_fin_extruded: fin_pitch <= fin_thickness (no air gap)");
    }
    if (in.base_thickness_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "heatsink_blade_fin_extruded: base_thickness_mm cannot be negative");
    }
    if (in.fin_count >= 2 && in.fin_pitch_mm > 0.0 && in.fin_thickness_mm > 0.0) {
        const double spanY = (in.fin_count - 1) * in.fin_pitch_mm + in.fin_thickness_mm;
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        if (spanY > (yMax - yMin) + 1e-3) {
            r.add("DFM-FIN-FIT", "error",
                  "heatsink_blade_fin_extruded: array span " +
                  std::to_string(spanY) + " mm > base width " +
                  std::to_string(yMax - yMin) + " mm");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "heatsink_blade_fin_extruded DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zTop = zMax;

    constexpr double kEmbed = 0.05;
    std::vector<TopoDS_Shape> additions;

    // Optional base plate.
    double basePlateVol = 0.0;
    double zFinBase = zTop;
    if (in.base_thickness_mm > 0.0) {
        const double spanY =
            (in.fin_count - 1) * in.fin_pitch_mm + in.fin_thickness_mm;
        const gp_Pnt baseOrigin(in.origin_x_mm,
                                in.origin_y_mm,
                                zTop - kEmbed);
        const gp_Ax2 baseAx(baseOrigin, gp::DZ());
        additions.push_back(pr::box(baseAx,
                                    in.fin_length_mm,
                                    spanY,
                                    in.base_thickness_mm + kEmbed));
        basePlateVol = in.fin_length_mm * spanY * in.base_thickness_mm;
        zFinBase = zTop + in.base_thickness_mm;
    }

    // N parallel blade fins (along +X, spaced along +Y).
    for (int i = 0; i < in.fin_count; ++i) {
        const gp_Pnt origin(
            in.origin_x_mm,
            in.origin_y_mm + i * in.fin_pitch_mm,
            zFinBase - kEmbed);
        const gp_Ax2 ax(origin, gp::DZ());
        additions.push_back(pr::box(ax,
                                    in.fin_length_mm,
                                    in.fin_thickness_mm,
                                    in.fin_height_mm + kEmbed));
    }

    const TopoDS_Shape newShape = pr::fuseMany(wp.shape(), additions);

    const double singleFinVol =
        in.fin_length_mm * in.fin_thickness_mm * in.fin_height_mm;
    const double finsVol = singleFinVol * in.fin_count;
    const double addedVol = basePlateVol + finsVol;

    json params = {
        { "face_id",            in.face_id },
        { "base_thickness_mm",  in.base_thickness_mm },
        { "fin_count",          in.fin_count },
        { "fin_pitch_mm",       in.fin_pitch_mm },
        { "fin_thickness_mm",   in.fin_thickness_mm },
        { "fin_height_mm",      in.fin_height_mm },
        { "fin_length_mm",      in.fin_length_mm },
        { "origin_x_mm",        in.origin_x_mm },
        { "origin_y_mm",        in.origin_y_mm },
    };
    json pattern_js = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "electronics_feature_type", "blade_fin_heat_sink" },
        { "subfeature_count",       in.fin_count + (in.base_thickness_mm > 0.0 ? 1 : 0) },
        { "fin_count",              in.fin_count },
        { "fin_thickness_mm",       in.fin_thickness_mm },
        { "fin_height_mm",          in.fin_height_mm },
        { "fin_pitch_mm",           in.fin_pitch_mm },
        { "fin_length_mm",          in.fin_length_mm },
        { "base_thickness_mm",      in.base_thickness_mm },
        { "fin_aspect_ratio",       in.fin_height_mm / std::max(1e-9, in.fin_thickness_mm) },
        { "derived_volume_added_mm3", addedVol },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "extrusion;mill";
    tooling.tool_dia_mm       = in.fin_thickness_mm;
    tooling.tool_length_mm    = in.fin_height_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 500.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = -addedVol;
    tooling.est_cycle_time_s  = std::max(5.0, in.fin_count * 2.0);
    tooling.extra = {
        { "operation_note",
          "Standard extruded heat sink stock, or mill blade-fin array from solid plate." },
        { "fin_count",       in.fin_count },
        { "added_volume_mm3", addedVol },
    };

    FeatureSignature sig { kSkillId, params, pattern_js, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::heatsink_blade_fin_extruded applied: {} fins ({}x{}x{} mm), base {} mm",
        in.fin_count, in.fin_length_mm, in.fin_thickness_mm,
        in.fin_height_mm, in.base_thickness_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata replay primary; geometric back-map counts parallel ±Y planar
// wall faces (fin sides) above the wp bbox top.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 0.95;
        r.matched_geometry = {
            { "source",      "metadata_replay" },
            { "is_compound", true },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric: count parallel walls (±Y normal) above original wp top.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    int wallCount = 0;
    double sumHeight = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(n.Y()) < 0.9 || std::abs(n.X()) > 0.1 ||
            std::abs(n.Z()) > 0.1) continue;
        Bnd_Box bb; BRepBndLib::Add(wp.face(i), bb);
        if (bb.IsVoid()) continue;
        double a0, b0, c0, a1, b1, c1;
        bb.Get(a0, b0, c0, a1, b1, c1);
        if (c1 - c0 < 0.5) continue;
        ++wallCount;
        sumHeight += (c1 - c0);
    }
    if (wallCount < 4) return out;
    const int fins = wallCount / 2;
    const double meanH = sumHeight / std::max(1, wallCount);

    json recovered = {
        { "fin_count",      fins },
        { "fin_height_mm",  meanH },
        { "fin_thickness_mm", 0.0 },
        { "fin_pitch_mm",   0.0 },
        { "fin_length_mm",  0.0 },
    };
    json matched = {
        { "source",          "geometric_blade_walls" },
        { "wall_face_count", wallCount },
    };
    const double conf = (fins >= 3) ? 0.60 : 0.50;
    out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    return out;
}

}  // namespace koocadcam::skill::heatsink_blade_fin_extruded
