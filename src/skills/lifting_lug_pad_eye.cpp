// @lat: [[engine/skills#lifting_lug_pad_eye]]

#include "lifting_lug_pad_eye.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::lifting_lug_pad_eye {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.plate_t_mm <= 0.0 || in.pin_hole_dia_mm <= 0.0 ||
        in.lug_height_mm <= 0.0 || in.lug_width_mm <= 0.0 ||
        in.chamfer_mm < 0.0 || in.base_bevel_mm < 0.0)
    {
        r.add("DFM-LUG-INPUT", "error",
              "lifting_lug_pad_eye: all dims must be > 0 (chamfer/bevel may be 0)");
        return r;
    }

    // Pin centered at (width/2, height × 0.75) — top third of the lug.
    const double pinZ = in.lug_height_mm * 0.75;
    const double topEdge = in.lug_height_mm - pinZ;
    if (topEdge < 1.5 * in.pin_hole_dia_mm) {
        r.add("DFM-LUG-PIN-EDGE", "error",
              "lifting_lug_pad_eye: pin-to-top edge " + std::to_string(topEdge) +
              " mm < 1.5 × pin_dia (" + std::to_string(1.5 * in.pin_hole_dia_mm) +
              " mm) — DNV-OS-H205 padeye minimum");
    }
    if (in.plate_t_mm < in.pin_hole_dia_mm * 0.5) {
        r.add("DFM-LUG-T-PIN", "error",
              "lifting_lug_pad_eye: plate_t " + std::to_string(in.plate_t_mm) +
              " mm < pin_dia × 0.5 — bearing pressure too high");
    }
    const double ar = in.lug_width_mm / in.lug_height_mm;
    if (ar < 0.6 || ar > 1.5) {
        r.add("DFM-LUG-AR", "warning",
              "lifting_lug_pad_eye: aspect ratio w/h " + std::to_string(ar) +
              " outside [0.6, 1.5]");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lifting_lug_pad_eye DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double t  = in.plate_t_mm;
    const double Hh = in.lug_height_mm;
    const double Ww = in.lug_width_mm;
    const double Dp = in.pin_hole_dia_mm;

    // Sub-feature 1: plate body, centered in Y.
    const gp_Pnt origin(0.0, -t / 2.0, 0.0);
    TopoDS_Shape plate = pr::box(gp_Ax2(origin, gp::DZ()), Ww, t, Hh);

    // Sub-feature 2: shackle pin hole, axis along Y (through-thickness).
    const double pinX = Ww / 2.0;
    const double pinZ = Hh * 0.75;
    const gp_Ax2 pinAx(gp_Pnt(pinX, -t / 2.0 - 0.1, pinZ), gp::DY());
    const TopoDS_Shape pinHole = pr::cylinder(pinAx, Dp / 2.0, t + 0.2);
    plate = pr::cut(plate, pinHole);

    // Sub-feature 3: 45° entry-face chamfer around the hole (entry side = +Y face).
    // Use a cone frustum: r1 (at face) = pin_r + chamfer, r2 (deeper) = pin_r.
    int chamCount = 0;
    if (in.chamfer_mm > 0.0) {
        const double pinR    = Dp / 2.0;
        const double chamfR1 = pinR + in.chamfer_mm;
        const double chamfR2 = pinR;
        // Cone axis: start just outside +Y face, point into -Y.
        const gp_Ax2 chamfAx(
            gp_Pnt(pinX, t / 2.0 + 0.01, pinZ),
            gp_Dir(0.0, -1.0, 0.0));
        try {
            const TopoDS_Shape chamfer =
                pr::coneFrustum(chamfAx, chamfR1, chamfR2, in.chamfer_mm + 0.02);
            plate = pr::cut(plate, chamfer);
            chamCount = 1;
        } catch (...) {
            // Chamfer is decorative; fall back silently if OCCT fails.
        }
    }

    // Sub-feature 4: base weld-prep bevel.  Triangular cut at the base
    // (z = 0) on both Y faces — modeled as two box cutters offset/rotated.
    // Simpler: cut a single triangular prism that bevels the front-bottom edge.
    int bevelCount = 0;
    if (in.base_bevel_mm > 0.0) {
        // Cutter is a wedge box rotated 45° around the bottom edge.
        // We approximate: a thin axis-aligned box at base that we'll cut.
        // 45° base bevel: removes material in a triangular cross section
        // spanning base_bevel_mm in Y and base_bevel_mm in Z.
        // Front bevel.
        const double bv = in.base_bevel_mm;
        const gp_Pnt frontBevOrigin(-0.1, -t / 2.0 - 0.1, -0.1);
        const TopoDS_Shape frontBevBox = pr::box(
            gp_Ax2(frontBevOrigin, gp::DZ()), Ww + 0.2, bv + 0.1, bv + 0.1);
        try {
            plate = pr::cut(plate, frontBevBox);
            bevelCount = 1;
        } catch (...) {
            // Bevel optional; skip on failure.
        }
        const gp_Pnt rearBevOrigin(-0.1, t / 2.0 - bv, -0.1);
        const TopoDS_Shape rearBevBox = pr::box(
            gp_Ax2(rearBevOrigin, gp::DZ()), Ww + 0.2, bv + 0.1, bv + 0.1);
        try {
            plate = pr::cut(plate, rearBevBox);
            ++bevelCount;
        } catch (...) {
        }
    }

    const int subfeatureCount = 1 + 1 + chamCount + bevelCount;

    json params = {
        { "plate_t_mm",       t },
        { "pin_hole_dia_mm",  Dp },
        { "lug_height_mm",    Hh },
        { "lug_width_mm",     Ww },
        { "chamfer_mm",       in.chamfer_mm },
        { "base_bevel_mm",    in.base_bevel_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    subfeatureCount },
        { "plate_t_mm",          t },
        { "pin_hole_dia_mm",     Dp },
        { "lug_height_mm",       Hh },
        { "lug_width_mm",        Ww },
        { "pin_center",          { { "x", pinX }, { "z", pinZ } } },
        { "pin_top_edge_mm",     Hh - pinZ },
        { "has_chamfer",         chamCount > 0 },
        { "has_base_bevel",      bevelCount > 0 },
    };

    const double volBody = Ww * Hh * t;
    const double volPin  = M_PI * (Dp / 2.0) * (Dp / 2.0) * t;
    const double weightKg = (volBody - volPin) * 7.85e-6;

    ToolingMeta tooling;
    tooling.tool_type         = "plasma_cut;drill;chamfer_mill;bevel_mill";
    tooling.tool_dia_mm       = Dp;
    tooling.tool_length_mm    = t + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.06;
    tooling.stock_removed_mm3 = volPin;
    tooling.est_cycle_time_s  = 60.0;
    tooling.extra = {
        { "subfeature_sequence", {
            { { "name", "plate_body" } },
            { { "name", "pin_hole" }, { "dia_mm", Dp } },
            { { "name", "face_chamfer" }, { "size_mm", in.chamfer_mm } },
            { { "name", "base_bevel" }, { "size_mm", in.base_bevel_mm } },
        } },
        { "approx_weight_kg",    weightKg },
        { "standard",            "DNV-OS-H205_AWS_D1.1" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(plate, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::lifting_lug_pad_eye applied: w={} h={} t={} pin={}",
                  Ww, Hh, t, Dp);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

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
                                { "is_compound", true } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::lifting_lug_pad_eye
