// @lat: [[engine/skills#indexed_5ax_setup]]

#include "indexed_5ax_setup.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Frames.hpp"   // for M_PI fallback on MSVC

#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::indexed_5ax_setup {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.rotation_angle_deg < -360.0 || in.rotation_angle_deg > 360.0) {
        r.add("DFM-INDEX-ANGLE", "error",
              "indexed_5ax_setup rotation_angle_deg " +
              std::to_string(in.rotation_angle_deg) +
              " outside [-360, 360]");
    }
    // gp_Ax1 normalises its direction at construction so we don't need to
    // re-check magnitude; if a caller passed a zero-vector gp_Dir, OCCT
    // would have thrown before we got here.

    r.add("DFM-005", "info",
          "indexed_5ax_setup requires a 4th/5th rotary axis (indexed 3+2 mode)");
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "indexed_5ax_setup DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Build the rotation transform.
    const double angleRad = in.rotation_angle_deg * M_PI / 180.0;
    gp_Trsf trsf;
    trsf.SetRotation(in.rotation_axis, angleRad);

    // Apply with copy=true so the new shape doesn't alias the input.
    BRepBuilderAPI_Transform builder(wp.shape(), trsf, /*copy=*/true);
    if (!builder.IsDone())
        throw SkillError("indexed_5ax_setup: BRepBuilderAPI_Transform failed");
    const TopoDS_Shape newShape = builder.Shape();

    const gp_Pnt o = in.rotation_axis.Location();
    const gp_Dir d = in.rotation_axis.Direction();

    json params = {
        { "rotation_axis",        {
            { "origin",    { o.X(), o.Y(), o.Z() } },
            { "direction", { d.X(), d.Y(), d.Z() } },
        }},
        { "rotation_angle_deg",   in.rotation_angle_deg },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_setup_only",         true },     // no material removed
        { "rotation_angle_deg",    in.rotation_angle_deg },
        { "rotation_axis_dir",     { d.X(), d.Y(), d.Z() } },
        { "rotation_axis_origin",  { o.X(), o.Y(), o.Z() } },
        { "requires_rotary_axis",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    // No cutting tool — this is a setup operation.
    tooling.tool_type         = "rotary_table";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Indexing time scales with rotation angle; rule-of-thumb 90° / s.
    tooling.est_cycle_time_s  = std::max(1.0,
                                         std::abs(in.rotation_angle_deg) / 90.0);
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "3+2 indexed setup — rotates workpiece before next 3-axis pass";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    // Carry over prior features (this skill doesn't add a cut, but it
    // shouldn't drop history either).
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::indexed_5ax_setup applied: angle={:.2f} deg axis=({},{},{})",
                  in.rotation_angle_deg, d.X(), d.Y(), d.Z());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Cannot truly detect a rotation from final shape alone — there's no
// reference frame.  Heuristic: if the LARGEST planar face's outward normal
// is NOT aligned with +Z (i.e. it's not sitting "naturally" on the table),
// emit a low-confidence indexed_5ax_setup candidate with the rotation
// being a guess (we can't recover the exact axis without the prior shape).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Find the largest planar face and inspect its normal.
    int    bestIdx  = -1;
    double bestArea = 0.0;
    gp_Dir bestNorm(0, 0, 1);
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const double a = wp.faceArea(i);
        if (a > bestArea) {
            try {
                bestNorm = wp.faceNormal(i);
                bestArea = a;
                bestIdx  = i;
            } catch (...) {
                continue;
            }
        }
    }
    if (bestIdx < 0) return out;

    // Natural orientation: largest planar face's normal aligned with global
    // axis (typically ±Z).  If the largest face is along a non-global
    // direction, the workpiece may have been indexed.
    const double ax = std::abs(bestNorm.X());
    const double ay = std::abs(bestNorm.Y());
    const double az = std::abs(bestNorm.Z());
    const bool axisAligned = ax > 0.99 || ay > 0.99 || az > 0.99;
    if (axisAligned) return out;

    // Estimate rotation: assume the workpiece was originally Z-up; compute
    // the angle between (0,0,1) and bestNorm, around an inferred axis.
    const gp_Vec n0(0, 0, 1);
    const gp_Vec n1(bestNorm.X(), bestNorm.Y(), bestNorm.Z());
    gp_Vec axisVec = n0.Crossed(n1);
    if (axisVec.Magnitude() < 1e-9) return out;
    axisVec.Normalize();
    const double cosA = std::clamp(n0.Dot(n1), -1.0, 1.0);
    const double angleRad = std::acos(cosA);
    const double angleDeg = angleRad * 180.0 / M_PI;

    json recovered = {
        { "rotation_axis", {
            { "origin",    { 0.0, 0.0, 0.0 } },
            { "direction", { axisVec.X(), axisVec.Y(), axisVec.Z() } },
        }},
        { "rotation_angle_deg", angleDeg },
    };
    json matched = {
        { "largest_planar_face_id", bestIdx },
        { "largest_planar_normal",  { bestNorm.X(), bestNorm.Y(), bestNorm.Z() } },
        { "largest_planar_area",    bestArea },
    };
    out.push_back(RecognizedFeature{
        kSkillId, recovered, /*confidence*/ 0.30, matched
    });
    return out;
}

}  // namespace koocadcam::skill::indexed_5ax_setup
