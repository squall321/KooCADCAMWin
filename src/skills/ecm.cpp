// @lat: [[engine/skills#ecm]]

#include "ecm.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Fillets.hpp"
#include "engine/primitives/Tools.hpp"

#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::ecm {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "ecm diameter_mm must be > 0");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "ecm depth_mm must be > 0");
    }
    if (in.diameter_mm > 0.0 && in.diameter_mm < 1.0) {
        r.add("DFM-ECM-DIA", "error",
              "ecm diameter_mm " + std::to_string(in.diameter_mm) +
              " < 1.0 mm — electrolyte flow unstable below 1 mm gap diameter");
    }
    if (in.depth_mm > 25.0) {
        r.add("DFM-ECM-DEPTH", "warning",
              "ecm depth_mm " + std::to_string(in.depth_mm) +
              " > 25 mm — flush pump capacity / electrolyte stagnation risk");
    }
    if (in.corner_r_mm < 0.0) {
        r.add("DFM-INPUT", "error", "ecm corner_r_mm must be >= 0");
    }
    if (in.diameter_mm > 0.0 && in.corner_r_mm > in.diameter_mm / 2.0 - 1e-6) {
        r.add("DFM-ECM-CORNER", "error",
              "ecm corner_r_mm " + std::to_string(in.corner_r_mm) +
              " >= diameter/2 — cavity geometry would collapse");
    }
    if (in.depth_mm > 0.0 && in.corner_r_mm > in.depth_mm - 1e-6) {
        r.add("DFM-ECM-CORNER", "error",
              "ecm corner_r_mm must be < depth_mm");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ecm DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("ecm: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kEntryOverhang);
        }
    } else {
        const double bboxDiag = std::sqrt(
            (xMax - xMin) * (xMax - xMin) +
            (yMax - yMin) * (yMax - yMin) +
            (zMax - zMin) * (zMax - zMin));
        const double margin = bboxDiag + 1.0;
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * margin,
            in.position_y_mm - adir.Y() * margin,
            (zMin + zMax) / 2.0 - adir.Z() * margin);
    }

    const double toolHeight = in.depth_mm + kEntryOverhang;
    // gp_Ax2(P, V, Vx) — V is the local Z (axis), so the cylinder extrudes
    // along adir.
    const gp_Ax2 toolAx(toolStart, adir);
    TopoDS_Shape cutter = pr::cylinder(toolAx, in.diameter_mm / 2.0, toolHeight);

    // Apply bottom-edge fillet on the cutter so the resulting cavity has a
    // rounded bottom corner (characteristic ECM signature).
    if (in.corner_r_mm > 0.0) {
        const double zBottom = toolStart.Z() + adir.Z() * toolHeight;
        cutter = pr::filletEdges(cutter, in.corner_r_mm,
            [zBottom](const TopoDS_Edge& e, const gp_Pnt& mp) {
                (void)e;
                return std::abs(mp.Z() - zBottom) < 1e-3;
            });
    }

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { adir.X(), adir.Y(), adir.Z() } },
        { "diameter_mm",    in.diameter_mm },
        { "depth_mm",       in.depth_mm },
        { "corner_r_mm",    in.corner_r_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "toroidal_face_count",        in.corner_r_mm > 0.0 ? 1 : 0 },
        { "bottom_planar_face_present", true },
        { "diameter_mm",                in.diameter_mm },
        { "depth_mm",                   in.depth_mm },
        { "corner_r_mm",                in.corner_r_mm },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "ecm_cathode";
    tooling.tool_dia_mm       = in.diameter_mm;
    tooling.tool_length_mm    = in.depth_mm + 5.0;
    tooling.tool_material     = "copper";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = M_PI * (in.diameter_mm / 2.0) * (in.diameter_mm / 2.0)
                              * in.depth_mm;
    // ECM material removal rate ≈ 1.5 mm/min linear feed; cycle = depth/rate.
    tooling.est_cycle_time_s  = std::max(20.0, in.depth_mm * 40.0);
    tooling.extra["dfm_findings"]        = findings;
    tooling.extra["machining_constraint"] =
        "ECM — no HAZ, naturally rounded corners, conductive workpiece required";
    tooling.extra["electrolyte"] = "NaCl_aq";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::ecm applied: dia={} depth={} cornerR={} faces {}→{}",
                  in.diameter_mm, in.depth_mm, in.corner_r_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// ECM cavities are geometrically indistinguishable from a milled pocket
// with a corner fillet.  Without metadata we cannot distinguish ECM from
// `mill_circular_pocket` with `bottom_corner_r > 0`.  We therefore return
// only metadata-replay candidates (confidence 1.0) and zero otherwise.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::ecm
