// @lat: [[engine/skills#cooling_plate_channel_serpentine]]

#include "cooling_plate_channel_serpentine.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::cooling_plate_channel_serpentine {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "cooling_plate_channel_serpentine: face_id out of range");
    }
    if (!(in.channel_width_mm > 0.0) || !(in.channel_depth_mm > 0.0) ||
        !(in.channel_pitch_mm > 0.0) || !(in.straight_length_mm > 0.0) ||
        in.channel_count < 2) {
        r.add("DFM-INPUT", "error",
              "cooling_plate_channel_serpentine: dimensions > 0, "
              "channel_count ≥ 2 required");
        return r;
    }

    const double aspect = in.channel_width_mm / in.channel_depth_mm;
    if (aspect < 0.4 || aspect > 5.0) {
        r.add("DFM-ASPECT", "error",
              "cooling_plate_channel_serpentine: width/depth ratio " +
              std::to_string(aspect) +
              " outside [0.4, 5.0] manufacturable range");
    }

    if (in.channel_pitch_mm < in.channel_width_mm + 1.5) {
        r.add("DFM-RIB-WALL", "error",
              "cooling_plate_channel_serpentine: channel_pitch must exceed "
              "channel_width by ≥ 1.5 mm (rib wall for coolant pressure)");
    }

    if (in.straight_length_mm < 3.0 * in.channel_width_mm) {
        r.add("DFM-STRAIGHT-LEN", "warning",
              "cooling_plate_channel_serpentine: straight_length < 3 × width — "
              "unusually short for serpentine cooling efficiency");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cooling_plate_channel_serpentine DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const double baseZ = faceC.Z();

    // Cut direction = INTO the plate (opposite outward normal).
    const gp_Dir cutDir(-n.X(), -n.Y(), -n.Z());

    const double W   = in.channel_width_mm;
    const double D   = in.channel_depth_mm;
    const double L   = in.straight_length_mm;
    const double P   = in.channel_pitch_mm;
    const double x0  = in.inlet_x_mm;
    const double y0  = in.inlet_y_mm;
    constexpr double kEmbed = 0.05;
    const double cutDepth = D + kEmbed;

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(static_cast<size_t>(in.channel_count * 2 + 2));
    double totalRemoved = 0.0;

    // ── Straights: each runs along +Y or -Y, alternating direction ───────
    // straight i sits at X = x0 + i * P, spans Y in [y0, y0 + L]
    for (int i = 0; i < in.channel_count; ++i) {
        const double straightX = x0 + i * P;
        // Box: DX = W (along X), DY = L (along Y), DZ = cutDepth (along cutDir).
        // To use box(ax, dx, dy, dz) with Vmain = cutDir, Vx = gp::DX().
        const gp_Pnt sOrigin(straightX - W / 2.0, y0, baseZ + kEmbed);
        const gp_Ax2 sAx(sOrigin, cutDir, gp::DX());
        // box(ax, dx, dy, dz): dx along Vx (X), dy along Vx × V = DX × cutDir.
        //   cutDir ≈ -Z → DX × (-Z) = +Y.  So DY along +Y as desired.
        cutters.push_back(pr::box(sAx, W, L, cutDepth));
        totalRemoved += W * L * D;
    }

    // ── 180° turns: connect straight i ↔ straight i+1 at alternating ends ─
    // turn j connects straight j to j+1.  If j is even, turn is at the TOP
    // (Y = y0 + L); if odd, at the BOTTOM (Y = y0).  Turn modelled as a
    // simple annular-ring cut centered between the two straights.
    for (int j = 0; j < in.channel_count - 1; ++j) {
        const double turnX = x0 + (j + 0.5) * P;        // midpoint between straights
        const double turnY = ((j % 2) == 0) ? (y0 + L) : y0;
        const double R_outer = P / 2.0 + W / 2.0;
        const double R_inner = P / 2.0 - W / 2.0;

        // Approximate the 180° connector as a full annular ring centered on
        // (turnX, turnY).  Half of it sits over the next-channel direction
        // and is the actual flow path; the other half is outside the
        // straights and won't intersect material outside our footprint —
        // but to be tidy, we instead approximate the turn with a
        // SHORT cross-tube: a box from straight i to straight i+1 at the
        // turn end, which is simpler and reliable booleanly.
        const double crossBoxW = P + W;  // spans both straights
        const double crossBoxH = W;       // axial along Y at the turn
        const double sign = ((j % 2) == 0) ? -1.0 : +1.0;
        // For even j (top turn): box centred at (turnX, turnY - W/2),
        //   DX = crossBoxW, DY = W (toward -Y so it inset back into straights).
        // For odd j (bottom turn): box centred at (turnX, turnY + W/2),
        //   DX = crossBoxW, DY = W toward +Y.
        const gp_Pnt tOrigin(
            turnX - crossBoxW / 2.0,
            turnY + ((sign < 0) ? -W : 0.0),
            baseZ + kEmbed);
        const gp_Ax2 tAx(tOrigin, cutDir, gp::DX());
        cutters.push_back(pr::box(tAx, crossBoxW, crossBoxH, cutDepth));
        totalRemoved += crossBoxW * crossBoxH * D;

        (void)R_outer; (void)R_inner;  // reserved for future arc-based turn
    }

    // ── Inlet stub + outlet stub ─────────────────────────────────────────
    // Inlet: extension off the first straight, lateral by W.
    {
        const double stubLen = W * 2.0;
        const double sx = x0 - W / 2.0 - stubLen;
        const gp_Pnt iOrigin(sx, y0, baseZ + kEmbed);
        const gp_Ax2 iAx(iOrigin, cutDir, gp::DX());
        cutters.push_back(pr::box(iAx, stubLen + W, W, cutDepth));
        totalRemoved += stubLen * W * D;
    }
    {
        // Outlet at the last straight; lateral the other way.
        const double stubLen = W * 2.0;
        const double lastX = x0 + (in.channel_count - 1) * P;
        const double lastY = ((in.channel_count - 1) % 2 == 0) ? (y0 + L) : y0;
        const double oxStart = lastX + W / 2.0;
        // Stub goes +X (outward).
        const gp_Pnt oOrigin(oxStart - W, lastY - W / 2.0, baseZ + kEmbed);
        const gp_Ax2 oAx(oOrigin, cutDir, gp::DX());
        cutters.push_back(pr::box(oAx, stubLen + W, W, cutDepth));
        totalRemoved += stubLen * W * D;
        (void)lastY;
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",            in.face_id },
        { "channel_width_mm",   in.channel_width_mm },
        { "channel_depth_mm",   in.channel_depth_mm },
        { "channel_pitch_mm",   in.channel_pitch_mm },
        { "channel_count",      in.channel_count },
        { "straight_length_mm", in.straight_length_mm },
        { "inlet_x_mm",         in.inlet_x_mm },
        { "inlet_y_mm",         in.inlet_y_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_ev_feature",       true },
        { "ev_feature_type",     "cooling_plate_channel" },
        { "channel_count",       in.channel_count },
        { "subfeature_count",    in.channel_count + (in.channel_count - 1) + 2 },
        { "channel_width_mm",    in.channel_width_mm },
        { "channel_depth_mm",    in.channel_depth_mm },
        { "channel_pitch_mm",    in.channel_pitch_mm },
        { "is_serpentine",       true },
        { "derived_volume_removed_mm3", totalRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = in.channel_width_mm;
    tooling.tool_length_mm    = in.channel_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = totalRemoved;
    tooling.est_cycle_time_s  = std::max(10.0, totalRemoved / 250.0);
    tooling.extra = {
        { "feature_standard", "EV serpentine liquid-cooling channel" },
        { "channel_count",    in.channel_count },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::cooling_plate_channel_serpentine applied: N={} W={} D={} P={}",
        in.channel_count, in.channel_width_mm, in.channel_depth_mm,
        in.channel_pitch_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + serpentine geometric fallback ─────────

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
            { "source",          "metadata_replay" },
            { "is_compound",     true },
            { "ev_feature_type", "cooling_plate_channel" },
        };
        out.push_back(r);
    }

    // Geometric fallback: count narrow planar wall pairs (a serpentine
    // channel leaves parallel side-wall faces).  If ≥ 4 such faces with
    // similar Y-spans exist, emit a candidate.
    if (out.empty()) {
        int planarSideCount = 0;
        for (int i = 0; i < wp.faceCount(); ++i) {
            if (!wp.isFacePlanar(i)) continue;
            const gp_Dir nf = wp.faceNormal(i);
            // Side walls of a Z-deep channel have horizontal normals.
            if (std::abs(nf.Z()) < 0.1) planarSideCount++;
        }
        if (planarSideCount >= 4) {
            json recovered = { { "channel_count", planarSideCount / 2 } };
            json matched = {
                { "source", "geometric_fallback" },
                { "planar_side_wall_count", planarSideCount },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.4, matched });
        }
    }
    return out;
}

}  // namespace koocadcam::skill::cooling_plate_channel_serpentine
