// @lat: [[engine/skills#vent_burst_disc_pocket]]

#include "vent_burst_disc_pocket.hpp"

#include "_as568_table.hpp"
#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

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

namespace koocadcam::skill::vent_burst_disc_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "vent_burst_disc_pocket: face_id out of range");
    }
    if (!(in.disc_dia_mm > 0.0) || !(in.pocket_depth_mm > 0.0) ||
        !(in.vent_dia_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "vent_burst_disc_pocket: all dimensions must be > 0");
        return r;
    }
    if (in.disc_dia_mm < 4.0 || in.disc_dia_mm > 25.0) {
        r.add("DFM-DISC-DIA", "error",
              "vent_burst_disc_pocket: disc_dia " +
              std::to_string(in.disc_dia_mm) + " outside [4, 25] mm");
    }

    const auto* spec = as568::findDash(in.o_ring_size_key);
    if (!spec) {
        r.add("DFM-AS568-KEY", "error",
              "vent_burst_disc_pocket: o_ring_size_key '" +
              in.o_ring_size_key + "' not in central AS568 table");
        return r;
    }

    if (in.pocket_depth_mm < spec->groove_depth_mm + 0.5) {
        r.add("DFM-POCKET-DEPTH", "warning",
              "vent_burst_disc_pocket: pocket_depth " +
              std::to_string(in.pocket_depth_mm) +
              " < O-ring groove depth + 0.5 mm seat clearance");
    }

    if (in.vent_dia_mm >= in.disc_dia_mm / 2.0) {
        r.add("DFM-VENT-OVERSIZE", "error",
              "vent_burst_disc_pocket: vent_dia must be < disc_dia / 2 "
              "(burst disc must overlap the vent)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "vent_burst_disc_pocket DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = as568::findDash(in.o_ring_size_key);

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const double baseZ = faceC.Z();
    const gp_Dir cutDir(-n.X(), -n.Y(), -n.Z());

    const double discR  = in.disc_dia_mm / 2.0;
    const double ventR  = in.vent_dia_mm / 2.0;
    const double grooveW = spec->groove_width_mm;
    const double grooveD = spec->groove_depth_mm;
    constexpr double kEmbed = 0.05;

    // O-ring groove sits OUTSIDE the disc bore, with a 0.5 mm gap:
    //   groove inner R = discR + 0.5
    //   groove outer R = groove inner R + groove_width
    const double grooveR_inner = discR + 0.5;
    const double grooveR_outer = grooveR_inner + grooveW;
    const double grooveMeanDia = grooveR_inner + grooveR_outer;

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(3);

    // ── Sub-feature 1: cylindrical pocket bore for disc ──────────────────
    const gp_Pnt pocketOrigin(in.center_x_mm, in.center_y_mm, baseZ + kEmbed);
    const gp_Ax2 pocketAx(pocketOrigin, cutDir);
    cutters.push_back(pr::cylinder(pocketAx, discR, in.pocket_depth_mm + kEmbed));
    const double pocketVol = M_PI * discR * discR * in.pocket_depth_mm;

    // ── Sub-feature 2: AS568 O-ring face groove (annular) ────────────────
    const gp_Pnt grooveOrigin(in.center_x_mm, in.center_y_mm, baseZ + kEmbed);
    const gp_Ax2 grooveAx(grooveOrigin, cutDir);
    cutters.push_back(pr::annularRing(
        grooveAx, grooveR_outer, grooveR_inner, grooveD + kEmbed));
    const double grooveVol = M_PI *
        (grooveR_outer * grooveR_outer - grooveR_inner * grooveR_inner) *
        grooveD;

    // ── Sub-feature 3: through vent hole at centre ───────────────────────
    // We size the through depth using the workpiece bounding box thickness.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thru = (zMax - zMin) + 2.0;
    const gp_Pnt ventOrigin(in.center_x_mm, in.center_y_mm, baseZ + kEmbed);
    const gp_Ax2 ventAx(ventOrigin, cutDir);
    cutters.push_back(pr::cylinder(ventAx, ventR, thru));
    const double ventVol = M_PI * ventR * ventR * thru;

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    const double totalRemoved = pocketVol + grooveVol + ventVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",         in.face_id },
        { "center_x_mm",     in.center_x_mm },
        { "center_y_mm",     in.center_y_mm },
        { "disc_dia_mm",     in.disc_dia_mm },
        { "pocket_depth_mm", in.pocket_depth_mm },
        { "o_ring_size_key", in.o_ring_size_key },
        { "vent_dia_mm",     in.vent_dia_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_ev_feature",       true },
        { "ev_feature_type",     "vent_burst_disc" },
        { "subfeature_count",    3 },
        { "disc_dia_mm",         in.disc_dia_mm },
        { "pocket_depth_mm",     in.pocket_depth_mm },
        { "o_ring_size_key",     in.o_ring_size_key },
        { "o_ring_groove_depth_mm", grooveD },
        { "o_ring_groove_width_mm", grooveW },
        { "groove_mean_dia_mm",  grooveMeanDia },
        { "vent_dia_mm",         in.vent_dia_mm },
        { "derived_volume_removed_mm3", totalRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;form_tool;drill";
    tooling.tool_dia_mm       = in.disc_dia_mm;
    tooling.tool_length_mm    = thru + 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = totalRemoved;
    tooling.est_cycle_time_s  = std::max(5.0, totalRemoved / 150.0);
    tooling.extra = {
        { "feature_standard", "EV pressure-relief burst-disc seat (AS568 face seal)" },
        { "o_ring_size_key",  in.o_ring_size_key },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::vent_burst_disc_pocket applied: discD={} oring={} ventD={}",
        in.disc_dia_mm, in.o_ring_size_key, in.vent_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay ─────────────────────────────────────────

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
            { "ev_feature_type", "vent_burst_disc" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::vent_burst_disc_pocket
