// @lat: [[engine/skills#flow_form]]

#include "flow_form.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::flow_form {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.outer_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "flow_form outer_dia_mm must be > 0 (got " +
              std::to_string(in.outer_dia_mm) + ")");
    }
    if (in.original_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "flow_form original_thickness_mm must be > 0 (got " +
              std::to_string(in.original_thickness_mm) + ")");
    }
    if (in.final_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "flow_form final_thickness_mm must be > 0 (got " +
              std::to_string(in.final_thickness_mm) + ")");
    }
    if (in.original_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "flow_form original_length_mm must be > 0 (got " +
              std::to_string(in.original_length_mm) + ")");
    }
    if (in.pass_count < 1) {
        r.add("DFM-INPUT", "error",
              "flow_form pass_count must be ≥ 1");
    }

    // Reduction sanity.
    if (in.original_thickness_mm > 0.0 && in.final_thickness_mm > 0.0) {
        if (in.final_thickness_mm >= in.original_thickness_mm) {
            r.add("DFM-INPUT", "error",
                  "flow_form final_thickness " +
                  std::to_string(in.final_thickness_mm) +
                  " mm >= original_thickness " +
                  std::to_string(in.original_thickness_mm) +
                  " mm — flow form REDUCES thickness");
        } else if (in.final_thickness_mm < in.original_thickness_mm * 0.2) {
            r.add("DFM-FF-MIN", "error",
                  "flow_form final_thickness " +
                  std::to_string(in.final_thickness_mm) +
                  " mm < 0.2 × original (" +
                  std::to_string(in.original_thickness_mm * 0.2) +
                  " mm) — > 80% reduction requires anneal between passes");
        }
    }

    // Workpiece must be tube-like (have an outer + an inner radius).  We
    // detect this by requiring TWO Z-axis cylindrical faces of differing
    // radius.  Single-radius (solid bar) cannot flow-form without pre-
    // piercing.
    if (!wp.shape().IsNull()) {
        int zCylCount = 0;
        for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
            if (!wp.isFaceCylinder(fIdx)) continue;
            BRepAdaptor_Surface s(wp.face(fIdx));
            const gp_Cylinder cyl = s.Cylinder();
            if (std::abs(std::abs(cyl.Axis().Direction().Z()) - 1.0) > 1e-3)
                continue;
            ++zCylCount;
        }
        if (zCylCount < 2) {
            r.add("DFM-FF-TUBE", "error",
                  "flow_form requires a tubular preform with both OD and ID — "
                  "found " + std::to_string(zCylCount) +
                  " Z-axis cylindrical face(s); pierce or bore first");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Strategy: rebuild as a new hollow cylinder.  Outer diameter stays the
// same; wall thickness goes from t0 → t1; length grows in proportion
// L1 = L0 × t0 / t1 (volume preservation, ignoring the OD growth).
//
// Implementation: build outer cylinder − inner cylinder using primitives.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "flow_form DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double t0 = in.original_thickness_mm;
    const double t1 = in.final_thickness_mm;
    const double L0 = in.original_length_mm;
    const double L1 = L0 * t0 / t1;          // volume preservation

    const double outerR = in.outer_dia_mm / 2.0;
    const double innerR = std::max(0.05, outerR - t1);

    TopoDS_Shape result;
    try {
        const gp_Ax2 ax(gp_Pnt(0.0, 0.0, 0.0), gp::DZ());
        result = pr::annularRing(ax, outerR, innerR, L1);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("flow_form: tube rebuild failed: ") + ex.what());
    }

    const double elongationPct = (L1 - L0) / L0 * 100.0;
    const double reductionPct  = (1.0 - t1 / t0) * 100.0;

    // Volume preserved → no removal.
    json params = {
        { "outer_dia_mm",           in.outer_dia_mm },
        { "original_thickness_mm",  t0 },
        { "final_thickness_mm",     t1 },
        { "original_length_mm",     L0 },
        { "pass_count",             in.pass_count },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "axis",                   { 0.0, 0.0, 1.0 } },
        { "outer_dia_mm",           in.outer_dia_mm },
        { "original_thickness_mm",  t0 },
        { "final_thickness_mm",     t1 },
        { "original_length_mm",     L0 },
        { "final_length_mm",        L1 },
        { "elongation_pct",         elongationPct },
        { "reduction_pct",          reductionPct },
        { "geometry_changed",       true },
        { "tubular",                true },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "flow_form_roller";
    tooling.tool_dia_mm       = 60.0;
    tooling.tool_length_mm    = L1;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;             // forming, not cutting
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;             // volume preserved
    tooling.est_cycle_time_s  = std::max(30.0,
        L1 / 4.0 * in.pass_count + 20.0);
    tooling.extra = json{
        { "process",               "flow_forming" },
        { "elongation_pct",        elongationPct },
        { "reduction_pct",         reductionPct },
        { "final_length_mm",       L1 },
        { "pass_count",            in.pass_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::flow_form applied: t0={} → t1={} ({}%) L0={} → L1={}",
                  t0, t1, reductionPct, L0, L1);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // (a) metadata replay.
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

    // (b) geometric fallback: detect a tubular shape (two Z-axis cylinders).
    if (wp.shape().IsNull()) return out;
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double L = zMax - zMin;
    const double D = std::max(xMax - xMin, yMax - yMin);
    if (L <= 0.0 || D <= 0.0) return out;

    // A flow-formed tube is long relative to its diameter (L/D > 1.5).
    if (L < 1.5 * D) return out;

    double rOuter = 0.0;
    double rInner = 0.0;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface s(wp.face(fIdx));
        const gp_Cylinder cyl = s.Cylinder();
        if (std::abs(std::abs(cyl.Axis().Direction().Z()) - 1.0) > 1e-3)
            continue;
        const double r = cyl.Radius();
        if (r > rOuter) { rInner = rOuter; rOuter = r; }
        else if (r > rInner && r < rOuter) { rInner = r; }
    }
    if (rOuter <= 0.0 || rInner <= 0.0 || rOuter <= rInner) return out;

    const double t = rOuter - rInner;
    json recovered = {
        { "outer_dia_mm",           2.0 * rOuter },
        { "original_thickness_mm",  0.0 },           // unknown
        { "final_thickness_mm",     t },
        { "original_length_mm",     0.0 },           // unknown
        { "pass_count",             1 },
    };
    json matched = {
        { "current_length_mm",      L },
        { "current_thickness_mm",   t },
        { "L_over_D",               L / (2.0 * rOuter) },
        { "source",                 "geometric_fallback" },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.35, matched });
    return out;
}

}  // namespace koocadcam::skill::flow_form
