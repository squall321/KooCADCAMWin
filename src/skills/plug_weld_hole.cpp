// @lat: [[engine/skills#plug_weld_hole]]

#include "plug_weld_hole.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

namespace koocadcam::skill::plug_weld_hole {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.hole_dia < 0.8) {
        r.add("DFM-002", "error",
              "plug_weld_hole: hole_dia " + std::to_string(in.hole_dia) +
              " mm < min 0.8 mm");
    }
    if (in.hole_dia < 3.0 || in.hole_dia > 12.0) {
        r.add("DFM-WELD-PLUG-DIA", "error",
              "plug_weld_hole: hole_dia " + std::to_string(in.hole_dia) +
              " mm outside [3, 12] (AWS A2.4 plug-weld range)");
    }
    if (in.cs_angle_deg < 45.0 || in.cs_angle_deg > 120.0) {
        r.add("DFM-WELD-CS-ANGLE", "error",
              "plug_weld_hole: cs_angle_deg " + std::to_string(in.cs_angle_deg) +
              "° outside [45°, 120°]");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "plug_weld_hole DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId)
        throw SkillError("plug_weld_hole: entry_face unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const gp_Dir adir = in.axis_dir;
    constexpr double kOverhang = 0.05;

    // Common case: drilling along ±Z.
    double entryZ = zMax;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        entryZ = (adir.Z() < 0) ? zMax : zMin;
    }

    // Cone top dia: caller may pass 0 → default hole_dia + 2 mm.
    const double csTopDia =
        (in.cs_top_dia > 0.0) ? in.cs_top_dia : (in.hole_dia + 2.0);
    if (csTopDia <= in.hole_dia)
        throw SkillError("plug_weld_hole: cs_top_dia <= hole_dia");

    const double halfRad =
        (in.cs_angle_deg * 0.5) * M_PI / 180.0;
    const double t = std::tan(halfRad);
    if (t < 1e-9)
        throw SkillError("plug_weld_hole: degenerate cs_angle");
    const double csDepth = (csTopDia - in.hole_dia) * 0.5 / t;

    // ── Sub-feature 1: through cylinder hole ──────────────────────────
    const gp_Pnt cylStart(
        in.position_x_mm,
        in.position_y_mm,
        entryZ - adir.Z() * kOverhang);
    const gp_Ax2 cylAx(cylStart, adir);
    const double cylH = bboxDiag + 2.0 * kOverhang;
    const TopoDS_Shape holeCyl = pr::cylinder(cylAx, in.hole_dia / 2.0, cylH);

    // ── Sub-feature 2: cone frustum countersink (entry face side) ─────
    const gp_Pnt coneOrigin(
        in.position_x_mm, in.position_y_mm, entryZ);
    const gp_Ax2 coneAx(coneOrigin, adir);
    const TopoDS_Shape csCone = pr::coneFrustum(
        coneAx, csTopDia / 2.0, in.hole_dia / 2.0, csDepth);

    // ── Fuse concentric cutters, then single cut ──────────────────────
    const TopoDS_Shape fused    = pr::fuse(holeCyl, csCone);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { adir.X(), adir.Y(), adir.Z() } },
        { "hole_dia",       in.hole_dia },
        { "cs_top_dia",     csTopDia },
        { "cs_angle_deg",   in.cs_angle_deg },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "is_compound",      true },
        { "subfeature_count", 2 },
        { "hole_dia",         in.hole_dia },
        { "cs_top_dia",       csTopDia },
        { "cs_angle_deg",     in.cs_angle_deg },
        { "cs_depth_mm",      csDepth },           // derived
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;countersink";
    tooling.tool_dia_mm       = csTopDia;
    tooling.tool_length_mm    = 15.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double holeR  = in.hole_dia / 2.0;
    const double csTopR = csTopDia    / 2.0;
    tooling.stock_removed_mm3 =
        M_PI * holeR * holeR * (zMax - zMin) +
        (M_PI * csDepth / 3.0) *
            (csTopR * csTopR + csTopR * holeR + holeR * holeR);
    tooling.est_cycle_time_s  = std::max(2.0, (zMax - zMin) / 50.0 + 1.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill"       }, { "tool_dia_mm", in.hole_dia } },
            { { "tool_type", "countersink" }, { "tool_dia_mm", csTopDia    },
              { "cone_angle_deg", in.cs_angle_deg } },
        } },
        { "aws_classification", "plug_weld" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::plug_weld_hole applied: dia={} cs_top={} angle={}",
                  in.hole_dia, csTopDia, in.cs_angle_deg);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& s : wp.features()) {
        if (s.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = s.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::plug_weld_hole
