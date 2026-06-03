// @lat: [[engine/skills#subgate_pin_point]]

#include "subgate_pin_point.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::subgate_pin_point {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Default runner-side diameter for the taper top (where the sub-gate meets
// the runner) when not explicitly given by the caller.  Slice-1 picks
// 3 × gate_dia clamped to a reasonable physical range.
double runnerTopDiameter(double gate_dia_mm)
{
    return std::clamp(gate_dia_mm * 3.0, 1.5, 6.0);
}

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!(in.gate_dia_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "subgate_pin_point: gate_dia_mm must be > 0");
    } else if (in.gate_dia_mm < 0.3 || in.gate_dia_mm > 2.0) {
        r.add("DFM-SUBGATE-DIA", "error",
              "subgate_pin_point: gate_dia " +
              std::to_string(in.gate_dia_mm) +
              " mm outside [0.3, 2.0] mm typical");
    }
    if (!(in.land_length_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "subgate_pin_point: land_length_mm must be > 0");
    } else if (in.land_length_mm < 0.5) {
        r.add("DFM-SUBGATE-LAND", "error",
              "subgate_pin_point: land_length " +
              std::to_string(in.land_length_mm) +
              " < 0.5 mm clearance minimum");
    }
    if (in.taper_angle_deg < 3.0 || in.taper_angle_deg > 15.0) {
        r.add("DFM-SUBGATE-TAPER", "error",
              "subgate_pin_point: taper_angle " +
              std::to_string(in.taper_angle_deg) +
              " deg outside [3, 15] range");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "subgate_pin_point DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto bb = pr::optimalBbox(wp.shape());
    const double topZ = bb.zMax;

    const double rGate   = in.gate_dia_mm / 2.0;
    const double rRunner = runnerTopDiameter(in.gate_dia_mm) / 2.0;

    // Taper height: ΔR / tan(taper_angle).
    const double tanA = std::tan(in.taper_angle_deg * M_PI / 180.0);
    if (tanA < 1e-9) {
        throw SkillError("subgate_pin_point: taper angle too small");
    }
    const double taperH = std::max(0.1, (rRunner - rGate) / tanA);

    constexpr double kOverhang = 0.05;
    const gp_Dir downward(0.0, 0.0, -1.0);

    std::vector<TopoDS_Shape> cutters;

    // 1. Cone — top at parting line (rRunner) → bottom at -taperH (rGate).
    {
        const gp_Pnt top(in.cavity_entry_x_mm,
                         in.cavity_entry_y_mm,
                         topZ + kOverhang);
        const gp_Ax2 ax(top, downward);
        cutters.push_back(pr::coneFrustum(ax, rRunner, rGate, taperH + kOverhang));
    }
    // 2. Cylindrical land — bottom at -taperH, extending down by land_length.
    {
        const gp_Pnt landTop(in.cavity_entry_x_mm,
                             in.cavity_entry_y_mm,
                             topZ - taperH + kOverhang);
        const gp_Ax2 ax(landTop, downward);
        cutters.push_back(pr::cylinder(ax, rGate, in.land_length_mm + kOverhang));
    }

    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape  = pr::cutMany(wp.shape(), cutters);
    if (newShape.IsNull()) {
        throw SkillError("subgate_pin_point: cutMany returned null shape");
    }
    const double volAfter  = volumeOf(newShape);
    const double removed   = volBefore - volAfter;

    json params = {
        { "cavity_entry_x_mm", in.cavity_entry_x_mm },
        { "cavity_entry_y_mm", in.cavity_entry_y_mm },
        { "gate_dia_mm",       in.gate_dia_mm },
        { "land_length_mm",    in.land_length_mm },
        { "taper_angle_deg",   in.taper_angle_deg },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "mold_feature_type",      "subgate_pin_point" },
        { "derived_volume_removed", removed },
        { "taper_height_mm",        taperH },
        { "runner_top_dia_mm",      2.0 * rRunner },
        { "volume_before_mm3",      volBefore },
        { "volume_after_mm3",       volAfter },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "mold_gate";
    tooling.tool_dia_mm       = in.gate_dia_mm;
    tooling.stock_removed_mm3 = removed;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::subgate_pin_point applied: gate_dia={} taper_h={} land={} removed={}",
                  in.gate_dia_mm, taperH, in.land_length_mm, removed);

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
        rf.confidence       = 0.92;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::subgate_pin_point
