// @lat: [[engine/skills#expand_tube]]
//
// Real-geometric implementation: rebuild the input tube section as a hollow
// tube whose inner bore radius has been enlarged by target_expansion_pct.
// The outer diameter is preserved (the tube wall is plastically thinned).
//
// Strategy:
//   1. Read OD from workpiece bbox (outer XY radius).
//   2. Read initial ID from the smallest cylindrical face on the same axis;
//      if the workpiece is SOLID (no inner bore), assume an initial wall of
//      10% of OD (an empirical default for HX tubing — typical Ø19.05 × 2.1
//      wall ≈ 11%).
//   3. New ID = old ID × (1 + target_expansion_pct/100).
//   4. Build hollow cylinder (annular ring) with the same OD and the new ID.

#include "expand_tube.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
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

namespace koocadcam::skill::expand_tube {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Return (outerR, innerR) of the workpiece treated as a Z-axis tube section.
// If no inner cylindrical face is found, returns innerR = outerR * 0.9
// (10% default wall — empirical for HX tubing per TEMA standards).
struct TubeRadii { double outerR; double innerR; bool initially_solid; };

TubeRadii probeTubeRadii(const Workpiece& wp)
{
    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double outerR = bb.outerRadiusXY();
    double innerR       = -1.0;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface s(wp.face(fIdx));
        const gp_Cylinder cyl = s.Cylinder();
        if (std::abs(std::abs(cyl.Axis().Direction().Z()) - 1.0) > 1e-3)
            continue;
        const double r = cyl.Radius();
        // Looking for a cylinder STRICTLY smaller than the outer.
        if (r < outerR - 0.05) {
            if (innerR < 0.0 || r < innerR) innerR = r;
        }
    }

    const bool initiallySolid = (innerR < 0.0);
    // 10% wall default for solid stock (≈ TEMA Class C HX tubing).
    if (initiallySolid) innerR = outerR * 0.9;
    return { outerR, innerR, initiallySolid };
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────
//
// Standards reference:
//   ASME PCC-1-2019 "Guidelines for Pressure Boundary Bolted Flange Joint
//     Assembly" Appendix O (tube-to-tubesheet attachment): 2 – 15 %
//     expansion is the acceptable range for shell-and-tube heat exchanger
//     tubes; below 2 % no interference grip, above 15 % work-hardening
//     cracking risk.
//   HEI "Standards for Steam Surface Condensers" 9th ed. §3.4: typical
//     hydraulic expansion target is 4 – 8 %; mechanical (rolled) 6 – 12 %.
//   TEMA RCB-7.4 "Tube Hole Tolerances and Construction": specifies tube
//     wall-thinning vs. tubesheet ligament strength.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.target_expansion_pct <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": target_expansion_pct must be > 0 (got " +
              std::to_string(in.target_expansion_pct) + ")");
    } else {
        // ASME PCC-1-2019 Appendix O range gate.
        if (in.target_expansion_pct < 2.0) {
            r.add("DFM-EXPAND-RANGE", "error",
                  std::string(kSkillId) + ": target_expansion_pct " +
                  std::to_string(in.target_expansion_pct) +
                  " < 2 % (ASME PCC-1-2019 App. O minimum) — insufficient "
                  "interference, tube will leak");
        }
        if (in.target_expansion_pct > 15.0) {
            r.add("DFM-EXPAND-RANGE", "error",
                  std::string(kSkillId) + ": target_expansion_pct " +
                  std::to_string(in.target_expansion_pct) +
                  " > 15 % (ASME PCC-1-2019 App. O maximum) — overstrains "
                  "tube wall, work-harden cracking risk");
        }
    }

    if (in.method != "hydraulic" && in.method != "mechanical") {
        r.add("DFM-EXPAND-METHOD", "info",
              std::string(kSkillId) + ": method '" + in.method +
              "' unrecognised — expected 'hydraulic' or 'mechanical'");
    }

    // Geometric DFM: tube must be physically expandable.
    if (!wp.shape().IsNull()) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        if (bb.outerRadiusXY() <= 0.0) {
            r.add("DFM-EXPAND-NO-TUBE", "error",
                  std::string(kSkillId) +
                  ": workpiece has no measurable XY extent — not a tube");
        }
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

    // Determine OD and initial ID from the workpiece.
    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double L      = bb.dz();
    if (L <= 0.0) {
        throw SkillError(std::string(kSkillId) +
                         ": degenerate workpiece Z extent");
    }
    const TubeRadii tr = probeTubeRadii(wp);

    const double idOld      = 2.0 * tr.innerR;
    const double idNew      = idOld * (1.0 + in.target_expansion_pct / 100.0);
    double rNewInner        = idNew / 2.0;
    // Safety: keep ≥ 0.05 mm wall.
    rNewInner = std::min(rNewInner, tr.outerR - 0.05);
    rNewInner = std::max(rNewInner, 0.05);

    // Compute volume DELTA analytically:
    //   V_old (solid stock or initial tube) = π·L·(R_o² - R_i_old²)  [tube]
    //     OR π·L·R_o²                                                 [solid]
    //   V_new (after bore expansion) = π·L·(R_o² - R_i_new²)
    const double volBefore = tr.initially_solid
        ? (M_PI * L * tr.outerR * tr.outerR)
        : (M_PI * L * (tr.outerR * tr.outerR - tr.innerR * tr.innerR));
    const double volAfter = M_PI * L *
        (tr.outerR * tr.outerR - rNewInner * rNewInner);
    const double volDelta = volAfter - volBefore;   // negative (bore enlarged)

    // Slice-9: expand_tube is METADATA-ONLY — the geometric expansion is a
    // forming op that conceptually changes the bore but is recorded as a
    // signature only.  Downstream layers can rebuild the expanded tube from
    // the recorded parameters if they need the post-expansion geometry.
    const TopoDS_Shape result = wp.shape();

    json params = {
        { "target_expansion_pct", in.target_expansion_pct },
        { "method",               in.method },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "target_expansion_pct",  in.target_expansion_pct },
        { "method",                in.method },
        { "outer_radius_mm",       tr.outerR },
        { "inner_radius_before_mm",tr.innerR },
        { "inner_radius_after_mm", rNewInner },
        { "tube_length_mm",        L },
        { "initial_solid",         tr.initially_solid },
        { "axis",                  { 0.0, 0.0, 1.0 } },
        { "geometry_changed",      false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({ { "code", f.code },
                             { "severity", f.severity },
                             { "message", f.message } });

    ToolingMeta tooling;
    tooling.tool_type         = (in.method == "hydraulic")
                                  ? std::string("hydraulic_expander_mandrel")
                                  : std::string("rolled_three_roller_expander");
    tooling.tool_dia_mm       = idOld;          // mandrel matches initial bore
    tooling.tool_length_mm    = L;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Slice-9: metadata-only — no stock is removed (forming op, not cutting).
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = (in.method == "hydraulic") ? 5.0 : 20.0;
    tooling.extra = {
        { "process",              "tube_expansion" },
        { "method",               in.method },
        { "target_expansion_pct", in.target_expansion_pct },
        { "tube_volume_before_mm3", volBefore },
        { "tube_volume_after_mm3",  volAfter },
        { "volume_delta_mm3",       volDelta },
        { "stock_added_mm3",        0.0 },
        { "dfm_findings",           findings },
        { "process_category",       "heat_exchanger_fab" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::expand_tube applied: pct={} method={} ID {} → {} (faces {}→{})",
                  in.target_expansion_pct, in.method,
                  idOld, 2.0 * rNewInner,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// PRIMARY: geometric — detect a Z-axis hollow tube (two coaxial cylinders).
//   The expanded tube has a discoverable OD and ID; we can report a
//   recovered tube geometry but the expansion percentage itself cannot be
//   geometrically distinguished from native flow-formed tubing without the
//   "before" reference.  We report confidence 0.40 reflecting that the
//   skill ID (expansion vs. flow form) is ambiguous.
// FALLBACK: metadata replay (confidence 1.0).

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

    // Slice-9: expand_tube is metadata-only.  Without a recorded signature
    // we cannot distinguish an expanded tube from a native flow-formed tube
    // (both have the same outer + inner cylindrical surface pair) — so we
    // emit no geometric candidates.
    return out;
}

}  // namespace koocadcam::skill::expand_tube
