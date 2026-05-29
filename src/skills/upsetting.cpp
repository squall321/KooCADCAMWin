// @lat: [[engine/skills#upsetting]]

#include "upsetting.hpp"

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

namespace koocadcam::skill::upsetting {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.compression_ratio <= 1.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": compression_ratio " +
              std::to_string(in.compression_ratio) +
              " must be > 1.0 (upsetting SHORTENS the stock)");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double dx = xMax - xMin;
    const double dy = yMax - yMin;
    const double dz = zMax - zMin;
    if (dx <= 0.0 || dy <= 0.0 || dz <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": degenerate stock bbox");
        return r;
    }

    // Slenderness buckling rule: L0 / D0 > 2.5 with heavy compression buckles.
    const double L0 = dz;
    const double D0 = std::max(dx, dy);
    if (in.compression_ratio > 1.5 && L0 / D0 > 2.5) {
        r.add("DFM-UPS-BUCKLE", "error",
              std::string(kSkillId) + ": stock L/D " +
              std::to_string(L0 / D0) +
              " > 2.5 with compression_ratio " +
              std::to_string(in.compression_ratio) +
              " > 1.5 — slender column will buckle before upsetting");
    }

    if (in.compression_ratio > 3.0) {
        r.add("DFM-UPS-HEAVY", "info",
              std::string(kSkillId) + ": compression_ratio " +
              std::to_string(in.compression_ratio) +
              " > 3.0 — typically multi-stage with reheat between stages");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Volume preservation:
//   A0 · L0 = A1 · L1
//   With L1 = L0 / cratio  →  A1 = A0 · cratio
//   Uniform XY scaling by √cratio preserves the original aspect ratio.

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

    const double L0 = dz;
    const double L1 = L0 / in.compression_ratio;
    const double latScale = std::sqrt(in.compression_ratio);
    const double newDx = dx * latScale;
    const double newDy = dy * latScale;

    // Re-center the new XY extents on the original centroid so the part
    // doesn't drift in space.
    const double cx = (xMin + xMax) * 0.5;
    const double cy = (yMin + yMax) * 0.5;
    const double newXmin = cx - newDx * 0.5;
    const double newYmin = cy - newDy * 0.5;

    TopoDS_Shape result;
    try {
        const gp_Ax2 ax(gp_Pnt(newXmin, newYmin, zMin), gp::DZ());
        result = pr::box(ax, newDx, newDy, L1);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string(kSkillId) +
                         ": upset rebuild failed: " + ex.what());
    }

    json params = {
        { "compression_ratio", in.compression_ratio },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "compression_ratio",   in.compression_ratio },
        { "original_length_mm",  L0 },
        { "final_length_mm",     L1 },
        { "lateral_scale",       latScale },
        { "original_dx_mm",      dx },
        { "original_dy_mm",      dy },
        { "final_dx_mm",         newDx },
        { "final_dy_mm",         newDy },
        { "axis",                "z" },
        { "geometry_changed",    true },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "upset_press";
    tooling.tool_dia_mm       = std::max(newDx, newDy);
    tooling.tool_length_mm    = L1;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;        // volume preserved
    tooling.est_cycle_time_s  = 8.0 + in.compression_ratio * 4.0;
    tooling.extra["machining_constraint"] =
        "Upsetting — flat-die axial compression; L/D limit 2.5 to avoid buckling; "
        "lateral spread = √compression_ratio";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::upsetting applied: cratio={} L0={} → L1={} lat×={}",
                  in.compression_ratio, L0, L1, latScale);

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
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::upsetting
