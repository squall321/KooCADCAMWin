// @lat: [[engine/skills#dental_implant_emergence_taper]]

#include "dental_implant_emergence_taper.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::dental_implant_emergence_taper {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.abutment_top_dia_mm <= in.abutment_bottom_dia_mm) {
        r.add("DFM-DENTAL-TAPER", "error",
              "dental_implant_emergence_taper: top_dia " +
              std::to_string(in.abutment_top_dia_mm) +
              " mm must exceed bottom_dia " +
              std::to_string(in.abutment_bottom_dia_mm) +
              " mm (else no emergence taper)");
    }
    if (in.transition_length_mm <= 0.0) {
        r.add("DFM-DENTAL-TRANS-LEN", "error",
              "dental_implant_emergence_taper: transition_length_mm must be > 0");
    }
    if (in.screw_thread_size.empty() ||
        thread_table::findMetric(in.screw_thread_size) == nullptr) {
        r.add("DFM-DENTAL-THREAD", "error",
              "dental_implant_emergence_taper: screw_thread_size '" +
              in.screw_thread_size +
              "' not found in iso_thread_table (use M2.5/M3/M4/M5/M6/...)");
    }
    // Bottom dia must be > tap pilot for the thread to fit inside the bore.
    if (const auto* t = thread_table::findMetric(in.screw_thread_size)) {
        if (in.abutment_bottom_dia_mm > 0.0 &&
            in.abutment_bottom_dia_mm <= t->tap_pilot_dia_mm) {
            r.add("DFM-DENTAL-THREAD", "error",
                  "dental_implant_emergence_taper: abutment_bottom_dia " +
                  std::to_string(in.abutment_bottom_dia_mm) +
                  " mm ≤ tap pilot " + std::to_string(t->tap_pilot_dia_mm) +
                  " mm (" + in.screw_thread_size +
                  ") — bore breaks out of abutment wall");
        }
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "dental_implant_emergence_taper DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("dental_implant_emergence_taper: entry_face unresolved");

    const auto* t = thread_table::findMetric(in.screw_thread_size);
    const double pilotDia = t->tap_pilot_dia_mm;     // tap pilot for the screw

    const gp_Dir adir   = in.implant_axis_dir;
    const gp_Pnt origin = in.implant_axis_origin;

    const double kOverhang = 0.05;

    // Sub-feature 1: conical taper cut from top_dia (at entry plane) down
    // to bottom_dia over transition_length, along adir.
    const gp_Pnt taperOrigin(
        origin.X() - adir.X() * kOverhang,
        origin.Y() - adir.Y() * kOverhang,
        origin.Z() - adir.Z() * kOverhang);
    const gp_Ax2 taperAx(taperOrigin, adir);
    const TopoDS_Shape taperTool =
        pr::coneFrustum(taperAx,
                        in.abutment_top_dia_mm    / 2.0,
                        in.abutment_bottom_dia_mm / 2.0,
                        in.transition_length_mm + kOverhang);

    // Sub-feature 2: axial pilot bore (tap pilot dia) — sized for thread.
    // Extends through the abutment body using stock-bbox diagonal as a
    // very-long bore so it punches through to whatever is beneath the
    // taper.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));
    const gp_Ax2 boreAx(taperOrigin, adir);
    const TopoDS_Shape boreTool =
        pr::cylinder(boreAx, pilotDia / 2.0, bboxDiag + 2.0 * kOverhang);

    // Fuse concentric cutters, single cut.
    const TopoDS_Shape fused    = pr::fuse(taperTool, boreTool);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // Signature
    json params = {
        { "entry_face_id",       *entryId },
        { "implant_axis_dir",    { adir.X(), adir.Y(), adir.Z() } },
        { "implant_axis_origin", { origin.X(), origin.Y(), origin.Z() } },
        { "abutment_top_dia_mm",     in.abutment_top_dia_mm },
        { "abutment_bottom_dia_mm",  in.abutment_bottom_dia_mm },
        { "transition_length_mm",    in.transition_length_mm },
        { "screw_thread_size",       in.screw_thread_size },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    2 },
        { "medical_feature_type","dental_emergence_profile" },
        { "screw_size",          in.screw_thread_size },
        { "abutment_top_dia_mm",    in.abutment_top_dia_mm },
        { "abutment_bottom_dia_mm", in.abutment_bottom_dia_mm },
        { "transition_length_mm",   in.transition_length_mm },
        { "screw_thread_size",      in.screw_thread_size },
        { "screw_pilot_dia_mm",     pilotDia },
        { "screw_thread_pitch_mm",  t->pitch_mm },
        { "screw_thread_nominal_dia_mm", t->nominal_dia_mm },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
        { "implant_standard",    "ISO_14801_dental" },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "form_cutter;drill;tap";
    tooling.tool_dia_mm = in.abutment_top_dia_mm;
    tooling.tool_length_mm = in.transition_length_mm * 1.5 + 5.0;
    tooling.tool_material  = "carbide";
    tooling.flute_count    = 3;
    tooling.cutting_speed_sfm = 150.0;
    tooling.feed_per_tooth_mm = 0.02;
    const double rT = in.abutment_top_dia_mm / 2.0;
    const double rB = in.abutment_bottom_dia_mm / 2.0;
    const double frustVol = (M_PI * in.transition_length_mm / 3.0)
                          * (rT * rT + rT * rB + rB * rB);
    const double boreVol  = M_PI * (pilotDia / 2.0) * (pilotDia / 2.0)
                          * (zMax - zMin);
    tooling.stock_removed_mm3 = frustVol + boreVol;
    tooling.est_cycle_time_s  = std::max(1.0, in.transition_length_mm / 8.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::dental_implant_emergence_taper applied: top {} bot {} L {} thread {}",
                  in.abutment_top_dia_mm, in.abutment_bottom_dia_mm,
                  in.transition_length_mm, in.screw_thread_size);

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
        rf.matched_geometry = { { "source", "metadata" },
                                { "is_compound", true },
                                { "medical_feature_type",
                                  "dental_emergence_profile" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::dental_implant_emergence_taper
