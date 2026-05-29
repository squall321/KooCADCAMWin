// @lat: [[engine/skills#open_die_forge]]

#include "open_die_forge.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Tools.hpp"

#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::open_die_forge {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.reduction_pct <= 0.0 || in.reduction_pct > 100.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": reduction_pct " +
              std::to_string(in.reduction_pct) + " out of (0, 100]");
    }
    if (in.temp_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": temp_c must be > 0 (got " +
              std::to_string(in.temp_c) + ")");
    }

    if (in.reduction_pct > 80.0) {
        r.add("DFM-FORGE-OVER", "error",
              std::string(kSkillId) + ": reduction_pct " +
              std::to_string(in.reduction_pct) +
              " > 80% per heat — cracking risk; reheat between passes");
    }

    if (in.temp_c > 0.0 && in.temp_c < 700.0) {
        r.add("DFM-FORGE-COLD", "info",
              std::string(kSkillId) + ": temp_c " +
              std::to_string(in.temp_c) +
              " °C < 700 °C — below typical hot-forging window for steel");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double tStock = std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
    if (tStock <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": degenerate stock bbox");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double dx = xMax - xMin;
    const double dy = yMax - yMin;
    const double dz = zMax - zMin;

    const double t0 = std::min({ dx, dy, dz });
    const double t1 = t0 * (1.0 - in.reduction_pct / 100.0);

    std::string thicknessAxis = "z";
    if (dx == t0)      thicknessAxis = "x";
    else if (dy == t0) thicknessAxis = "y";

    TopoDS_Shape result;
    try {
        const gp_Ax2 ax(gp_Pnt(xMin, yMin, zMin), gp::DZ());
        if (thicknessAxis == "z") {
            result = pr::box(ax, dx, dy, t1);
        } else if (thicknessAxis == "x") {
            result = pr::box(ax, t1, dy, dz);
        } else {
            result = pr::box(ax, dx, t1, dz);
        }
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string(kSkillId) +
                         ": forging rebuild failed: " + ex.what());
    }

    const double effectiveRedPct = (t0 > 0.0) ? (1.0 - t1 / t0) * 100.0 : 0.0;

    json params = {
        { "reduction_pct", in.reduction_pct },
        { "temp_c",        in.temp_c },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "original_thickness_mm",   t0 },
        { "target_thickness_mm",     t1 },
        { "effective_reduction_pct", effectiveRedPct },
        { "thickness_axis",          thicknessAxis },
        { "temp_c",                  in.temp_c },
        { "shape_constrained",       false },
        { "geometry_changed",        true },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "flat_die_hammer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = std::max(dx, dy);
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;        // forming, not removal
    tooling.est_cycle_time_s  = 10.0 + effectiveRedPct * 0.4;
    tooling.extra["machining_constraint"] =
        "Open-die forging — flat / V-dies; unconstrained spread; "
        "reheat between heavy reductions";
    tooling.extra["temp_c"] = in.temp_c;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::open_die_forge applied: t0={} → t1={} ({}%) @ {}°C",
                  t0, t1, effectiveRedPct, in.temp_c);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay (highest confidence).
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    if (wp.shape().IsNull()) return out;
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double dx = xMax - xMin;
    const double dy = yMax - yMin;
    const double dz = zMax - zMin;
    const double t  = std::min({ dx, dy, dz });
    if (t <= 0.0) return out;

    // Geometric fallback: a stout block (smallest extent ≤ 20 mm, but not a
    // sheet i.e. > 5 mm) MAY have been open-die forged.  Low confidence.
    if (t < 5.0 || t > 20.0) return out;

    std::string thicknessAxis = "z";
    if (dx == t)      thicknessAxis = "x";
    else if (dy == t) thicknessAxis = "y";

    json recovered = {
        { "reduction_pct", 0.0 },
        { "temp_c",        0.0 },
    };
    json matched = {
        { "thickness_axis",       thicknessAxis },
        { "current_thickness_mm", t },
        { "source",               "geometric_fallback" },
        { "bbox_x_mm",            dx },
        { "bbox_y_mm",            dy },
        { "bbox_z_mm",            dz },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.25, matched });
    return out;
}

}  // namespace koocadcam::skill::open_die_forge
