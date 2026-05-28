// @lat: [[engine/skills#rolling]]

#include "rolling.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::rolling {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

double currentThickness(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.target_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "rolling target_thickness_mm must be > 0");
    }
    if (in.reduction_pct_per_pass <= 0.0 || in.reduction_pct_per_pass > 100.0) {
        r.add("DFM-INPUT", "error",
              "rolling reduction_pct_per_pass " +
              std::to_string(in.reduction_pct_per_pass) +
              " out of (0, 100]");
    }
    if (in.reduction_pct_per_pass > 50.0) {
        r.add("DFM-ROLL-PASS", "error",
              "rolling reduction " + std::to_string(in.reduction_pct_per_pass) +
              "% > 50% per pass — industrial typical limit");
    }

    const double t0 = currentThickness(wp);
    if (t0 <= 0.0) {
        r.add("DFM-INPUT", "error", "rolling: degenerate stock thickness");
        return r;
    }
    if (in.target_thickness_mm > 0.0) {
        if (in.target_thickness_mm >= t0) {
            r.add("DFM-INPUT", "error",
                  "rolling target_thickness " + std::to_string(in.target_thickness_mm) +
                  " mm >= current thickness " + std::to_string(t0) +
                  " mm — rolling REDUCES thickness");
        } else if (in.target_thickness_mm < t0 * 0.1) {
            r.add("DFM-ROLL-MIN", "error",
                  "rolling target " + std::to_string(in.target_thickness_mm) +
                  " mm < 0.1 × original (" + std::to_string(t0 * 0.1) +
                  " mm) — max cold-roll reduction of 90% exceeded");
        } else {
            const double effectiveRedPct =
                (1.0 - in.target_thickness_mm / t0) * 100.0;
            if (effectiveRedPct > in.reduction_pct_per_pass + 1e-6) {
                r.add("DFM-ROLL-PASS", "error",
                      "rolling: target requires " +
                      std::to_string(effectiveRedPct) +
                      "% reduction in single pass > declared " +
                      std::to_string(in.reduction_pct_per_pass) + "%");
            }
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Strategy: identify the thinnest bbox axis as the thickness direction.
// Rebuild the workpiece as a new box with the same XY extents and target
// thickness along that axis.  This sidesteps OCCT's lack of strict
// non-uniform scaling and produces a clean topological result.
//
// For workpieces where Z is NOT the thinnest axis (an X- or Y-rolled
// sheet), we still operate on the Z axis for slice-1 — but report the
// thinness axis in the signature.  In practice, all KooCADCAM sheet stock
// has Z as the thickness axis.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "rolling DFM failed:";
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
    const double t1 = in.target_thickness_mm;

    // Identify which axis is the thickness axis.  For KooCADCAM stock,
    // Z is the canonical thickness — we always reduce along Z, replacing
    // the workpiece with a new box that has the same XY footprint but new
    // Z extent.  This is correct as long as Z is the thinnest axis (typical).
    //
    // If a non-Z axis is thinnest, we still build along Z but note the
    // discrepancy in the signature for downstream layers to handle.
    std::string thicknessAxis = "z";
    if (dx == t0)      thicknessAxis = "x";
    else if (dy == t0) thicknessAxis = "y";

    TopoDS_Shape result;
    try {
        if (thicknessAxis == "z") {
            // Preserve volume-style intent: keep XY same, set Z extent to t1.
            // Origin at (xMin, yMin, zMin); height = t1.
            const gp_Ax2 ax(gp_Pnt(xMin, yMin, zMin), gp::DZ());
            result = pr::box(ax, dx, dy, t1);
        } else if (thicknessAxis == "x") {
            const gp_Ax2 ax(gp_Pnt(xMin, yMin, zMin), gp::DZ());
            result = pr::box(ax, t1, dy, dz);
        } else {
            const gp_Ax2 ax(gp_Pnt(xMin, yMin, zMin), gp::DZ());
            result = pr::box(ax, dx, t1, dz);
        }
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("rolling: target sheet rebuild failed: ") + ex.what());
    }

    const double effectiveRedPct = (t0 > 0.0) ? (1.0 - t1 / t0) * 100.0 : 0.0;

    // Signature.
    json params = {
        { "target_thickness_mm",     in.target_thickness_mm },
        { "reduction_pct_per_pass",  in.reduction_pct_per_pass },
        { "original_thickness_mm",   t0 },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "original_thickness_mm",      t0 },
        { "target_thickness_mm",        t1 },
        { "effective_reduction_pct",    effectiveRedPct },
        { "thickness_axis",             thicknessAxis },
        { "parallel_planar_face_pair",  true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "roller_mill";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = std::max(dx, dy);
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // forming, not removal
    tooling.est_cycle_time_s  = 5.0 + effectiveRedPct * 0.2;
    tooling.extra["machining_constraint"] =
        "Rolling — paired rollers; cold rolling typical; volume preserved "
        "(length increases as thickness decreases)";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::rolling applied: t0={} → t1={} ({}% reduction)",
                  t0, t1, effectiveRedPct);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometrically, a rolled sheet is indistinguishable from a cuboid stock.
// Recognition is metadata-driven (look for prior `rolling` entries in the
// workpiece feature history) plus geometric sanity (two large parallel
// planar faces, sheet ≤ 5 mm).
//
// If no metadata, recognize() still returns a candidate for the smallest-
// extent direction with low confidence, marking original_thickness as 0.0.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = currentThickness(wp);
    if (t <= 0.0 || t > 5.0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double dx = xMax - xMin;
    const double dy = yMax - yMin;
    const double dz = zMax - zMin;

    std::string thicknessAxis = "z";
    if (dx == t)      thicknessAxis = "x";
    else if (dy == t) thicknessAxis = "y";

    // Check for prior rolling metadata in feature history.
    double recoveredOriginal = 0.0;
    double recoveredReductionPct = 0.0;
    bool   hasMetadata = false;
    for (const auto& f : wp.features()) {
        if (f.skill_id == kSkillId) {
            try {
                recoveredOriginal = f.pattern.value("original_thickness_mm", 0.0);
                recoveredReductionPct = f.pattern.value("effective_reduction_pct", 0.0);
                hasMetadata = true;
            } catch (...) { /* ignore */ }
        }
    }

    // Require either metadata-driven discovery (high confidence) or a
    // recognisable sheet shape (low confidence guess).
    json recovered = {
        { "target_thickness_mm",      t },
        { "reduction_pct_per_pass",   recoveredReductionPct },
        { "original_thickness_mm",    recoveredOriginal },
    };
    json matched = {
        { "thickness_axis",          thicknessAxis },
        { "current_thickness_mm",    t },
        { "metadata_present",        hasMetadata },
        { "bbox_x_mm",               dx },
        { "bbox_y_mm",               dy },
        { "bbox_z_mm",               dz },
    };
    const double confidence = hasMetadata ? 0.95 : 0.25;
    out.push_back(RecognizedFeature{ kSkillId, recovered, confidence, matched });
    return out;
}

}  // namespace koocadcam::skill::rolling
