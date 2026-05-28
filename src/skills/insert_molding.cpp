// @lat: [[engine/skills#insert_molding]]

#include "insert_molding.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <utility>

namespace koocadcam::skill::insert_molding {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.shell_thickness_mm <= 0.0) {
        r.add("DFM-INSERT-SHELL", "error",
              "insert_molding shell_thickness_mm must be > 0 (got " +
              std::to_string(in.shell_thickness_mm) + ")");
    } else if (in.shell_thickness_mm < 0.5) {
        r.add("DFM-INSERT-SHELL", "error",
              "insert_molding shell_thickness_mm " +
              std::to_string(in.shell_thickness_mm) +
              " < 0.5 mm — below practical mold-filling thickness");
    } else if (in.shell_thickness_mm > 10.0) {
        r.add("DFM-INSERT-SHELL", "error",
              "insert_molding shell_thickness_mm " +
              std::to_string(in.shell_thickness_mm) +
              " > 10 mm — sink marks / cycle time impractical; "
              "redesign with ribs instead");
    }

    if (in.insert_material.empty()) {
        r.add("DFM-INSERT-MAT", "error",
              "insert_molding: insert_material must be specified");
    }
    if (in.resin_material.empty()) {
        r.add("DFM-INSERT-MAT", "error",
              "insert_molding: resin_material must be specified");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Build an outer bbox grown by shell_thickness on every side, cut the
// original insert out of it (so the hollow inside hugs the insert), then
// fuse the resulting "skin" onto the insert.  Net workpiece geometry:
// insert + resin shell as a single bonded solid.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "insert_molding DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    if (wp.shape().IsNull())
        throw SkillError("insert_molding: workpiece (insert) shape is null");

    const auto bb = pr::optimalBbox(wp.shape());
    const double t = in.shell_thickness_mm;

    // Outer enclosing box: bbox grown by t on every side.
    const gp_Pnt outerOrigin(bb.xMin - t, bb.yMin - t, bb.zMin - t);
    const gp_Ax2 outerAx(outerOrigin, gp::DZ());
    const double ox = bb.dx() + 2.0 * t;
    const double oy = bb.dy() + 2.0 * t;
    const double oz = bb.dz() + 2.0 * t;
    const TopoDS_Shape outerBox = pr::box(outerAx, ox, oy, oz);

    // Hollow shell = outerBox - insert.  Then fuse onto the insert.
    const TopoDS_Shape shell    = pr::cut(outerBox, wp.shape());
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), shell);

    const double shellVol = ox * oy * oz - bb.dx() * bb.dy() * bb.dz();

    json params = {
        { "insert_material",     in.insert_material },
        { "resin_material",      in.resin_material },
        { "shell_thickness_mm",  in.shell_thickness_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "insert_material",       in.insert_material },
        { "resin_material",        in.resin_material },
        { "shell_thickness_mm",    in.shell_thickness_mm },
        { "additive",              true },
        { "encapsulates_insert",   true },
        { "process_family",        "injection_molding" },
        { "shell_bbox_volume_mm3", shellVol },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "insert_mold";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "P20_tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Volume ADDED (negative removed, matching weld_buildup convention).
    tooling.stock_removed_mm3 = -shellVol;
    tooling.est_cycle_time_s  = 0.0;       // mold-flow planner fills this in
    tooling.extra = {
        { "process",              "insert_molding" },
        { "process_family",       "injection_molding" },
        { "insert_material",      in.insert_material },
        { "resin_material",       in.resin_material },
        { "shell_thickness_mm",   in.shell_thickness_mm },
        { "dfm_findings",         findings },
    };

    FeatureSignature sig{ kSkillId, std::move(params), std::move(pattern),
                          std::move(tooling) };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::insert_molding applied: insert={} resin={} t={}mm shellVol={}",
                  in.insert_material, in.resin_material,
                  in.shell_thickness_mm, shellVol);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric recognition of a fused insert+shell is hard without access to
// the original insert shape — after the fuse the inner interface is no
// longer a free surface.  We replay history only (confidence 1.0).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "feature_history" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::insert_molding
