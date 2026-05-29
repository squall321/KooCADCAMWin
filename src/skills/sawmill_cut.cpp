// @lat: [[engine/skills#sawmill_cut]]

#include "sawmill_cut.hpp"

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

namespace koocadcam::skill::sawmill_cut {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

bool isValidCutType(const std::string& s)
{
    return s == "rip" || s == "cross_cut" || s == "resaw";
}

// Axis index along which the rebuild will shrink the workpiece.
//   rip       → X (0)
//   cross_cut → Y (1)
//   resaw     → Z (2)
int cutAxisIndex(const std::string& cut_type)
{
    if (cut_type == "rip")       return 0;
    if (cut_type == "cross_cut") return 1;
    return 2;  // resaw
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!isValidCutType(in.cut_type)) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": cut_type '" + in.cut_type +
              "' unknown — expected 'rip' | 'cross_cut' | 'resaw'");
    }
    if (in.cut_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": cut_thickness_mm must be > 0 (got " +
              std::to_string(in.cut_thickness_mm) + ")");
    }
    if (in.kerf_mm < 2.5 || in.kerf_mm > 8.0) {
        r.add("DFM-SAWMILL-KERF", "warning",
              std::string(kSkillId) + ": kerf_mm " +
              std::to_string(in.kerf_mm) +
              " outside typical sawmill band/circular range [2.5, 8.0] mm");
    }
    if (in.cut_thickness_mm > 0.0 && in.cut_thickness_mm < 5.0) {
        r.add("DFM-SAWMILL-MIN", "error",
              std::string(kSkillId) + ": cut_thickness_mm " +
              std::to_string(in.cut_thickness_mm) +
              " < 5 mm — board too thin for downstream milling/planing");
    }

    // Range against current bbox.
    if (isValidCutType(in.cut_type) && in.cut_thickness_mm > 0.0) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double extents[3] = { xMax - xMin, yMax - yMin, zMax - zMin };
        const int    axis = cutAxisIndex(in.cut_type);
        if (in.cut_thickness_mm >= extents[axis]) {
            r.add("DFM-SAWMILL-GROWTH", "error",
                  std::string(kSkillId) + ": cut_thickness_mm " +
                  std::to_string(in.cut_thickness_mm) +
                  " >= current extent " + std::to_string(extents[axis]) +
                  " mm along axis " + std::to_string(axis) +
                  " — sawmill REMOVES material");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sawmill_cut DFM failed:";
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
    const int axis = cutAxisIndex(in.cut_type);

    double newDx = dx, newDy = dy, newDz = dz;
    if      (axis == 0) newDx = in.cut_thickness_mm;
    else if (axis == 1) newDy = in.cut_thickness_mm;
    else                newDz = in.cut_thickness_mm;

    TopoDS_Shape result;
    try {
        const gp_Ax2 ax(gp_Pnt(xMin, yMin, zMin), gp::DZ());
        result = pr::box(ax, newDx, newDy, newDz);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("sawmill_cut: board rebuild failed: ") + ex.what());
    }

    const double removedExtent =
        (axis == 0) ? (dx - newDx)
      : (axis == 1) ? (dy - newDy)
      :               (dz - newDz);
    const double removedVolume =
        (axis == 0) ? removedExtent * dy * dz
      : (axis == 1) ? dx * removedExtent * dz
      :               dx * dy * removedExtent;

    json params = {
        { "cut_type",         in.cut_type },
        { "cut_thickness_mm", in.cut_thickness_mm },
        { "kerf_mm",          in.kerf_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "cut_type",             in.cut_type },
        { "cut_thickness_mm",     in.cut_thickness_mm },
        { "kerf_mm",              in.kerf_mm },
        { "original_extent_mm",   { dx, dy, dz } },
        { "result_extent_mm",     { newDx, newDy, newDz } },
        { "cut_axis_index",       axis },
        { "planar_face_count",    6 },
        { "geometry_kind",        "cuboid_board" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = (in.cut_type == "resaw") ? "bandsaw" : "circular_saw";
    tooling.tool_dia_mm       = (in.cut_type == "resaw") ? 0.0 : 400.0;   // 16" blade
    tooling.tool_length_mm    = std::max({ dx, dy, dz });
    tooling.tool_material     = "carbide_tipped_steel";
    tooling.flute_count       = (in.cut_type == "resaw") ? 4 : 40;        // teeth/inch · circumference
    tooling.cutting_speed_sfm = (in.cut_type == "resaw") ? 4000.0 : 10000.0;
    tooling.feed_per_tooth_mm = 0.10;
    tooling.stock_removed_mm3 = removedVolume;
    // ~30 mm/s through softwood; thicker stock = slower.
    tooling.est_cycle_time_s  = std::max(2.0, removedExtent / 30.0 + std::max(dy, dx) * 0.05);
    tooling.extra["machining_constraint"] =
        "Sawmill — wide kerf, fast bulk reduction; rough surface (needs planing).";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::sawmill_cut applied: type={} axis={} extent {}→{}",
                  in.cut_type, axis,
                  (axis == 0 ? dx : axis == 1 ? dy : dz),
                  in.cut_thickness_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// A sawn board is geometrically a cuboid — indistinguishable from any other
// rectangular stock.  Confidence is HIGH only when metadata is replayed.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay first.
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

    // Geometric fallback: any cuboid is a board candidate, low confidence.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double dx = xMax - xMin;
    const double dy = yMax - yMin;
    const double dz = zMax - zMin;
    if (dx <= 0.0 || dy <= 0.0 || dz <= 0.0) return out;

    // Pick the smallest extent as the inferred cut axis.
    int axis = 0;
    double minE = dx;
    if (dy < minE) { minE = dy; axis = 1; }
    if (dz < minE) { minE = dz; axis = 2; }
    const char* axisType = (axis == 0) ? "rip" : (axis == 1) ? "cross_cut" : "resaw";

    json recovered = {
        { "cut_type",         axisType },
        { "cut_thickness_mm", minE },
        { "kerf_mm",          4.0 },
    };
    json matched = {
        { "source",            "geometry_guess" },
        { "bbox_extent_mm",    { dx, dy, dz } },
        { "inferred_axis",     axis },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.25, matched });
    return out;
}

}  // namespace koocadcam::skill::sawmill_cut
