// @lat: [[engine/skills#overmolding]]

#include "overmolding.hpp"

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

namespace koocadcam::skill::overmolding {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.shell_thickness_mm <= 0.0) {
        r.add("DFM-OVMOLD-SHELL", "error",
              "overmolding shell_thickness_mm must be > 0 (got " +
              std::to_string(in.shell_thickness_mm) + ")");
    } else if (in.shell_thickness_mm < 0.5) {
        r.add("DFM-OVMOLD-SHELL", "error",
              "overmolding shell_thickness_mm " +
              std::to_string(in.shell_thickness_mm) +
              " < 0.5 mm — below practical TPE filling thickness");
    } else if (in.shell_thickness_mm > 6.0) {
        r.add("DFM-OVMOLD-SHELL", "error",
              "overmolding shell_thickness_mm " +
              std::to_string(in.shell_thickness_mm) +
              " > 6 mm — soft layer too thick (loses grip definition, "
              "sink-mark risk)");
    }

    if (in.durometer_shore_a < 20.0 || in.durometer_shore_a > 95.0) {
        r.add("DFM-OVMOLD-DUROM", "error",
              "overmolding durometer_shore_a " +
              std::to_string(in.durometer_shore_a) +
              " outside [20, 95] Shore A range");
    }

    if (in.substrate_material.empty()) {
        r.add("DFM-OVMOLD-MAT", "error",
              "overmolding: substrate_material must be specified");
    }
    if (in.overmold_material.empty()) {
        r.add("DFM-OVMOLD-MAT", "error",
              "overmolding: overmold_material must be specified");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Same geometric idiom as insert_molding: outer bbox grown by t, minus the
// substrate, fused back on.  Different signature kind + DFM tuned for soft
// elastomer overmolds.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "overmolding DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    if (wp.shape().IsNull())
        throw SkillError("overmolding: substrate shape is null");

    const auto bb = pr::optimalBbox(wp.shape());
    const double t = in.shell_thickness_mm;

    const gp_Pnt outerOrigin(bb.xMin - t, bb.yMin - t, bb.zMin - t);
    const gp_Ax2 outerAx(outerOrigin, gp::DZ());
    const double ox = bb.dx() + 2.0 * t;
    const double oy = bb.dy() + 2.0 * t;
    const double oz = bb.dz() + 2.0 * t;
    const TopoDS_Shape outerBox = pr::box(outerAx, ox, oy, oz);

    const TopoDS_Shape shell    = pr::cut(outerBox, wp.shape());
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), shell);

    const double shellVol = ox * oy * oz - bb.dx() * bb.dy() * bb.dz();

    json params = {
        { "substrate_material",   in.substrate_material },
        { "overmold_material",    in.overmold_material },
        { "durometer_shore_a",    in.durometer_shore_a },
        { "shell_thickness_mm",   in.shell_thickness_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "substrate_material",     in.substrate_material },
        { "overmold_material",      in.overmold_material },
        { "durometer_shore_a",      in.durometer_shore_a },
        { "shell_thickness_mm",     in.shell_thickness_mm },
        { "additive",               true },
        { "soft_outer_layer",       true },
        { "process_family",         "injection_molding" },
        { "shell_bbox_volume_mm3",  shellVol },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "overmold";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "P20_tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = -shellVol;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "process",            "overmolding" },
        { "process_family",     "injection_molding" },
        { "substrate_material", in.substrate_material },
        { "overmold_material",  in.overmold_material },
        { "durometer_shore_a",  in.durometer_shore_a },
        { "shell_thickness_mm", in.shell_thickness_mm },
        { "dfm_findings",       findings },
    };

    FeatureSignature sig{ kSkillId, std::move(params), std::move(pattern),
                          std::move(tooling) };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::overmolding applied: sub={} over={} durom={}A t={}mm",
                  in.substrate_material, in.overmold_material,
                  in.durometer_shore_a, in.shell_thickness_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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

}  // namespace koocadcam::skill::overmolding
