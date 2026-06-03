// @lat: [[engine/skills#body_combine_subtract]]

#include "body_combine_subtract.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::body_combine_subtract {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

TopoDS_Shape buildAux(const Input& in)
{
    const gp_Pnt origin(in.aux_origin[0], in.aux_origin[1], in.aux_origin[2]);
    const gp_Ax2 ax(origin, gp::DZ());
    if (in.aux_kind == "box") {
        return pr::box(ax, in.aux_dims[0], in.aux_dims[1], in.aux_dims[2]);
    }
    if (in.aux_kind == "cylinder") {
        return pr::cylinder(ax, in.aux_dims[0], in.aux_dims[1]);
    }
    throw SkillError("body_combine_subtract: unknown aux_kind '" + in.aux_kind + "'");
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.aux_kind != "box" && in.aux_kind != "cylinder") {
        r.add("DFM-INPUT", "error",
              "body_combine_subtract: aux_kind '" + in.aux_kind +
              "' must be 'box' or 'cylinder'");
    }
    if (in.aux_kind == "box") {
        if (in.aux_dims[0] <= 0.0 || in.aux_dims[1] <= 0.0 || in.aux_dims[2] <= 0.0)
            r.add("DFM-INPUT", "error",
                  "body_combine_subtract: box dims must all be > 0");
    } else if (in.aux_kind == "cylinder") {
        if (in.aux_dims[0] <= 0.0 || in.aux_dims[1] <= 0.0)
            r.add("DFM-INPUT", "error",
                  "body_combine_subtract: cylinder radius/height must be > 0");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "body_combine_subtract DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const TopoDS_Shape aux = buildAux(in);
    const double volBefore = volumeOf(wp.shape());
    const double volAux    = volumeOf(aux);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), aux);
    const double volAfter  = volumeOf(newShape);
    const double removed   = volBefore - volAfter;

    json params = {
        { "aux_kind",   in.aux_kind },
        { "aux_origin", in.aux_origin },
        { "aux_dims",   in.aux_dims },
    };
    json pattern = {
        { "kind",                          kSkillId },
        { "is_compound",                   false },
        { "aux_kind",                      in.aux_kind },
        { "aux_origin",                    in.aux_origin },
        { "aux_dims",                      in.aux_dims },
        { "aux_volume_mm3",                volAux },
        { "derived_removed_volume_mm3",    removed },
        { "volume_before_mm3",             volBefore },
        { "volume_after_mm3",              volAfter },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = (in.aux_kind == "cylinder") ? in.aux_dims[0] * 2.0 : 8.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = removed;
    tooling.est_cycle_time_s  = std::max(1.0, removed / 5000.0);

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::body_combine_subtract: vol {} → {} (-{})",
                  volBefore, volAfter, removed);

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
        rf.confidence       = 0.93;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::body_combine_subtract
