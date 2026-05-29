// @lat: [[engine/skills#wood_planing]]

#include "wood_planing.hpp"

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

namespace koocadcam::skill::wood_planing {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

double currentThickness(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

namespace {

// Return the axis index (0/1/2) of the smallest bbox extent.
int thicknessAxisIndex(double dx, double dy, double dz)
{
    int axis = 0;
    double minE = dx;
    if (dy < minE) { minE = dy; axis = 1; }
    if (dz < minE) { minE = dz; axis = 2; }
    return axis;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.removal_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": removal_mm must be > 0 (got " +
              std::to_string(in.removal_mm) + ")");
    }
    if (in.removal_mm > 3.0) {
        r.add("DFM-PLANE-PASS", "error",
              std::string(kSkillId) + ": removal_mm " +
              std::to_string(in.removal_mm) +
              " > 3.0 mm — single-pass tear-out + motor overload risk");
    } else if (in.removal_mm > 0.0 && in.removal_mm < 0.2) {
        r.add("DFM-PLANE-MIN", "warning",
              std::string(kSkillId) + ": removal_mm " +
              std::to_string(in.removal_mm) +
              " < 0.2 mm — knives may scrape rather than cut");
    }

    const double t0 = currentThickness(wp);
    if (t0 <= 0.0) {
        r.add("DFM-INPUT", "error", "wood_planing: degenerate stock thickness");
        return r;
    }
    if (in.removal_mm > 0.0) {
        const double remaining = t0 - in.removal_mm;
        if (remaining < 6.0) {
            r.add("DFM-PLANE-LEAVE", "error",
                  std::string(kSkillId) + ": post-plane thickness " +
                  std::to_string(remaining) +
                  " mm < 6 mm — board too thin to safely feed");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "wood_planing DFM failed:";
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

    const int axis = thicknessAxisIndex(dx, dy, dz);
    const char* axisLabel = (axis == 0) ? "x" : (axis == 1) ? "y" : "z";

    double newDx = dx, newDy = dy, newDz = dz;
    if      (axis == 0) newDx = dx - in.removal_mm;
    else if (axis == 1) newDy = dy - in.removal_mm;
    else                newDz = dz - in.removal_mm;

    TopoDS_Shape result;
    try {
        const gp_Ax2 ax(gp_Pnt(xMin, yMin, zMin), gp::DZ());
        result = pr::box(ax, newDx, newDy, newDz);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("wood_planing: board rebuild failed: ") + ex.what());
    }

    const double removedVolume =
        (axis == 0) ? in.removal_mm * dy * dz
      : (axis == 1) ? dx * in.removal_mm * dz
      :               dx * dy * in.removal_mm;
    const double targetThickness =
        (axis == 0) ? newDx : (axis == 1) ? newDy : newDz;

    json params = {
        { "removal_mm", in.removal_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "removal_mm",           in.removal_mm },
        { "thickness_axis",       axisLabel },
        { "original_thickness_mm",
          (axis == 0 ? dx : axis == 1 ? dy : dz) },
        { "target_thickness_mm",  targetThickness },
        { "surface_finish",       "smooth" },
        { "planar_face_count",    6 },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "wood_planer_head";
    tooling.tool_dia_mm       = 75.0;     // typical 3" cutterhead
    tooling.tool_length_mm    = std::max(dx, dy);
    tooling.tool_material     = "high_speed_steel";
    tooling.flute_count       = 3;        // 3-knife head common
    tooling.cutting_speed_sfm = 4000.0;
    tooling.feed_per_tooth_mm = 0.8;
    tooling.stock_removed_mm3 = removedVolume;
    // Planers run ~15 mm/s feed; pad ramp/feed.
    tooling.est_cycle_time_s  = std::max(2.0,
        std::max(dx, dy) / 15.0 + in.removal_mm * 0.5);
    tooling.extra["machining_constraint"] =
        "Wood planer — flattens & thicknesses; finishing pass should be ≤ 0.5 mm "
        "for smooth surface.";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::wood_planing applied: axis={} removal={} thickness={}->{}",
                  axisLabel, in.removal_mm,
                  (axis == 0 ? dx : axis == 1 ? dy : dz), targetThickness);

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
    if (!out.empty()) return out;

    // Geometric fallback (low confidence).
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double dx = xMax - xMin;
    const double dy = yMax - yMin;
    const double dz = zMax - zMin;
    if (dx <= 0.0 || dy <= 0.0 || dz <= 0.0) return out;

    const int axis = thicknessAxisIndex(dx, dy, dz);
    const char* axisLabel = (axis == 0) ? "x" : (axis == 1) ? "y" : "z";

    json recovered = {
        { "removal_mm", 0.0 },   // historical removal is unknowable from geometry
    };
    json matched = {
        { "source",          "geometry_guess" },
        { "bbox_extent_mm",  { dx, dy, dz } },
        { "thickness_axis",  axisLabel },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.25, matched });
    return out;
}

}  // namespace koocadcam::skill::wood_planing
