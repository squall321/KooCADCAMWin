// @lat: [[engine/skills#pulley_with_keyway_compound]]

#include "pulley_with_keyway_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::pulley_with_keyway_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.outer_dia_mm <= 0.0 || in.thickness_mm <= 0.0 ||
        in.bore_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pulley_with_keyway_compound: positive outer/thickness/bore required");
        return r;
    }

    if (in.bore_dia_mm < 6.0) {
        r.add("DFM-PULLEY-BORE", "error",
              "pulley_with_keyway_compound: bore_dia " +
              std::to_string(in.bore_dia_mm) +
              " mm < 6 mm (typical shaft minimum)");
    }
    if (in.bore_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "pulley_with_keyway_compound: bore_dia < 0.8 mm");
    }
    if (in.bore_dia_mm >= in.outer_dia_mm) {
        r.add("DFM-PULLEY-BORE", "error",
              "pulley_with_keyway_compound: bore_dia >= outer_dia");
    }

    const double wallThk = (in.outer_dia_mm - in.bore_dia_mm) / 2.0;
    if (wallThk < 5.0) {
        r.add("DFM-PULLEY-WALL", "error",
              "pulley_with_keyway_compound: wall thickness " +
              std::to_string(wallThk) + " mm < 5 mm minimum");
    }

    if (in.groove_angle_deg < 30.0 || in.groove_angle_deg > 50.0) {
        r.add("DFM-PULLEY-ANGLE", "error",
              "pulley_with_keyway_compound: groove_angle " +
              std::to_string(in.groove_angle_deg) +
              " deg outside SAE J1313 range [30, 50]");
    }

    if (in.groove_depth_mm > 0.0 && in.groove_depth_mm >= wallThk) {
        r.add("DFM-PULLEY-WALL", "error",
              "pulley_with_keyway_compound: groove_depth would breach bore wall");
    }

    if (in.key_w_mm <= 0.0 || in.key_h_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pulley_with_keyway_compound: key_w and key_h must be > 0");
    }
    if (in.key_w_mm >= in.bore_dia_mm / 2.0) {
        r.add("DFM-KEYWAY-FIT", "error",
              "pulley_with_keyway_compound: key_w " +
              std::to_string(in.key_w_mm) +
              " mm >= bore_dia/2 — keyway breaks the bore wall");
    }

    if (in.balance_drill_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "pulley_with_keyway_compound: balance drill < 0.8 mm");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pulley_with_keyway_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double outerR = in.outer_dia_mm / 2.0;
    const double boreR  = in.bore_dia_mm  / 2.0;
    const double Z      = in.thickness_mm;
    const double kOverhang = 0.05;

    // The stock is the supplied wp (caller passes cylindrical stock).  We
    // operate fully in OCCT booleans rather than rebuilding the stock — so
    // wp can be any cylindrical-puck-like solid.

    // ── Tool 1: through bore (cylinder along Z, full thickness + overhang)
    const gp_Ax2 axZ(gp_Pnt(0, 0, -kOverhang), gp::DZ());
    const TopoDS_Shape boreTool =
        pr::cylinder(axZ, boreR, Z + 2.0 * kOverhang);

    // ── Tool 2: V-groove (cone frustum-shaped ring around the equator)
    //
    // The groove is at z = Z/2.  We synthesize it as an annular ring at the
    // OD whose inner radius dips down by groove_depth (effectively a flat-
    // bottom approximation — real V is geometrically equivalent for our
    // metadata-tracking purposes).  The angle is recorded; for slice-1 we
    // model the groove as a plain rectangular annular ring of width derived
    // from the included angle: w = 2·depth·tan(angle/2).
    const double grooveDepth = (in.groove_depth_mm > 0.0)
        ? in.groove_depth_mm
        : std::min(2.0, (outerR - boreR) * 0.4);   // default 2 mm or 40% of wall
    const double angRad   = in.groove_angle_deg * M_PI / 180.0;
    const double grooveW  = 2.0 * grooveDepth * std::tan(angRad / 2.0);
    const double zGrooveBot = Z / 2.0 - grooveW / 2.0;
    const gp_Ax2 axGroove(gp_Pnt(0, 0, zGrooveBot), gp::DZ());
    // Outer ring slightly past OD so the cut surface is clean.
    const TopoDS_Shape grooveTool = pr::annularRing(
        axGroove, outerR + kOverhang, outerR - grooveDepth, grooveW);

    // ── Tool 3: keyway slot in the bore (rect box, along Z, breaks bore
    //           wall by key_h_mm radially)
    const double keyW = in.key_w_mm;
    const double keyH = in.key_h_mm;
    // box origin: -keyW/2 in X, +boreR in Y (sits at the +Y wall of the
    // bore, projecting outward by keyH).
    const gp_Pnt keyOrigin(-keyW / 2.0, boreR, -kOverhang);
    const gp_Ax2 keyAx(keyOrigin, gp::DZ());
    const TopoDS_Shape keyBox =
        pr::box(keyAx, keyW, keyH, Z + 2.0 * kOverhang);

    // ── Tool 4-5: two balance drills, 180° apart, on a pitch circle that
    //              sits halfway between bore and OD.
    const double pitchR = (boreR + outerR) / 2.0;
    const double bDia   = in.balance_drill_dia_mm;
    const gp_Pnt bp1(pitchR, 0.0, -kOverhang);
    const gp_Pnt bp2(-pitchR, 0.0, -kOverhang);
    const TopoDS_Shape bdrill1 =
        pr::cylinder(gp_Ax2(bp1, gp::DZ()), bDia / 2.0, Z + 2.0 * kOverhang);
    const TopoDS_Shape bdrill2 =
        pr::cylinder(gp_Ax2(bp2, gp::DZ()), bDia / 2.0, Z + 2.0 * kOverhang);

    // ── Fuse the bore + keyway (coaxial overlap) then add the other tools
    //     as a separate compound cut.  This keeps the boolean clean.
    const TopoDS_Shape boreFused   = pr::fuse(boreTool, keyBox);

    std::vector<TopoDS_Shape> tools = {
        boreFused, grooveTool, bdrill1, bdrill2
    };
    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), tools);

    // ── Signature
    json params = {
        { "outer_dia_mm",        in.outer_dia_mm },
        { "groove_angle_deg",    in.groove_angle_deg },
        { "groove_depth_mm",     grooveDepth },
        { "thickness_mm",        in.thickness_mm },
        { "key_w_mm",            in.key_w_mm },
        { "key_h_mm",            in.key_h_mm },
        { "bore_dia_mm",         in.bore_dia_mm },
        { "balance_drill_dia_mm", in.balance_drill_dia_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  5 },   // bore, V-groove, keyway, 2 balance drills
        { "outer_dia_mm",      in.outer_dia_mm },
        { "groove_angle_deg",  in.groove_angle_deg },
        { "groove_depth_mm",   grooveDepth },
        { "groove_width_mm",   grooveW },
        { "thickness_mm",      in.thickness_mm },
        { "key_w_mm",          in.key_w_mm },
        { "key_h_mm",          in.key_h_mm },
        { "bore_dia_mm",       in.bore_dia_mm },
        { "balance_drill_dia_mm", in.balance_drill_dia_mm },
        { "balance_drill_count",  2 },
        { "balance_pitch_r_mm",   pitchR },
    };
    ToolingMeta tooling;
    tooling.tool_type     = "lathe;form_tool;end_mill;drill";
    tooling.tool_dia_mm   = in.bore_dia_mm;
    tooling.tool_material = "carbide";
    tooling.stock_removed_mm3 =
        M_PI * boreR * boreR * Z +
        M_PI * grooveW * grooveDepth * (2.0 * outerR - grooveDepth) +
        keyW * keyH * Z +
        2.0 * M_PI * (bDia / 2.0) * (bDia / 2.0) * Z;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },     { "tool_dia_mm", in.bore_dia_mm } },
            { { "tool_type", "form_tool" }, { "note", "V-groove angle " +
                                              std::to_string(in.groove_angle_deg) + " deg" } },
            { { "tool_type", "end_mill" },  { "tool_dia_mm", in.key_w_mm },
              { "note", "keyway in bore" } },
            { { "tool_type", "drill" },     { "tool_dia_mm", in.balance_drill_dia_mm },
              { "pass_count", 2 }, { "note", "balance drills 180 deg apart" } },
        } },
    };

    FeatureSignature sig {
        kSkillId, params, pattern, tooling,
    };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::pulley_with_keyway_compound applied: OD={} bore={} key={}x{}",
                  in.outer_dia_mm, in.bore_dia_mm, in.key_w_mm, in.key_h_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Compound features are hard to fingerprint geometrically once they've
// gone through STEP round-trip — we therefore primarily replay from
// feature-history metadata.  Confidence = 1.0 when the signature is
// found in wp.features().

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",            "feature_history_replay" },
            { "subfeature_count",  f.pattern.value("subfeature_count", 0) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::pulley_with_keyway_compound
