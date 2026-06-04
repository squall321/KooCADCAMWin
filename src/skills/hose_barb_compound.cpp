// @lat: [[engine/skills#hose_barb_compound]]

#include "hose_barb_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::hose_barb_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.barb_count < 2 || in.barb_count > 8) {
        r.add("DFM-BARB-N", "error",
              "hose_barb_compound: barb_count " +
              std::to_string(in.barb_count) + " not in [2, 8]");
    }
    if (in.base_dia_mm <= 0.0 || in.barb_max_dia_mm <= 0.0 ||
        in.barb_pitch_mm <= 0.0 || in.taper_to_tip_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "hose_barb_compound: positive dimensions required");
    }
    if (in.barb_max_dia_mm <= in.base_dia_mm) {
        r.add("DFM-BARB-OD", "error",
              "hose_barb_compound: barb_max_dia must exceed base_dia (need ridge)");
    }
    if (!(std::abs(in.axis_dir.X()) < 1e-6 && std::abs(in.axis_dir.Y()) < 1e-6
          && in.axis_dir.Z() > 0)) {
        r.add("DFM-AXIS", "error",
              "hose_barb_compound: only axis_dir = +Z supported");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "hose_barb_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;
    const double zBase = zMax;
    const double baseR = in.base_dia_mm / 2.0;
    const double maxR  = in.barb_max_dia_mm / 2.0;

    // Total length of barb body: from base plate up through N ridges + tip.
    const double bodyLen = in.barb_count * in.barb_pitch_mm + in.taper_to_tip_mm;

    // ── 1) base cylindrical body (fuse #1) ─────────────────────────────────
    const gp_Pnt bodyStart(cx, cy, zBase);
    const gp_Ax2 bodyAx(bodyStart, gp::DZ());
    TopoDS_Shape current = pr::fuse(
        wp.shape(),
        pr::cylinder(bodyAx, baseR, bodyLen));

    // ── 2..(1+N) ridges (sequential fuses, each cone frustum) ─────────────
    //
    // Each ridge starts at z_i = zBase + i*barb_pitch with the widest part
    // (barb_max_dia) at the bottom and tapers DOWN to base_dia at the top
    // (so the next ridge starts at base_dia, creating the saw-tooth profile).
    for (int i = 0; i < in.barb_count; ++i) {
        const double z0 = zBase + i * in.barb_pitch_mm;
        const gp_Pnt ringOrigin(cx, cy, z0);
        const gp_Ax2 ringAx(ringOrigin, gp::DZ());
        const TopoDS_Shape ridge = pr::coneFrustum(
            ringAx, maxR, baseR, in.barb_pitch_mm);
        current = pr::fuse(current, ridge);
    }

    // ── (2+N): tip taper (final cone, base_dia → 0.6*base_dia) ────────────
    if (in.taper_to_tip_mm > 0.0) {
        const double tipZ = zBase + in.barb_count * in.barb_pitch_mm;
        const gp_Pnt tipOrigin(cx, cy, tipZ);
        const gp_Ax2 tipAx(tipOrigin, gp::DZ());
        const double tipR = baseR * 0.6;
        const TopoDS_Shape tipCone = pr::coneFrustum(
            tipAx, baseR, tipR, in.taper_to_tip_mm);
        current = pr::fuse(current, tipCone);
    }

    // ── Approx volumes (per cone frustum: π/3 * h * (R²+Rr+r²)) ─────────
    const double vBody = M_PI * baseR * baseR * bodyLen;
    double vRidges = 0.0;
    for (int i = 0; i < in.barb_count; ++i) {
        vRidges += M_PI / 3.0 * in.barb_pitch_mm
                   * (maxR * maxR + maxR * baseR + baseR * baseR);
    }
    const double vAdded = vBody + vRidges;

    json params = {
        { "center_x_mm",      cx },
        { "center_y_mm",      cy },
        { "axis_dir",         { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "base_dia_mm",      in.base_dia_mm },
        { "barb_count",       in.barb_count },
        { "barb_pitch_mm",    in.barb_pitch_mm },
        { "barb_max_dia_mm",  in.barb_max_dia_mm },
        { "taper_to_tip_mm",  in.taper_to_tip_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "hvac_feature_type", "hose_barb" },
        { "subfeature_count",  1 + in.barb_count
                               + (in.taper_to_tip_mm > 0.0 ? 1 : 0) },
        { "barb_count",        in.barb_count },
        { "barb_max_dia_mm",   in.barb_max_dia_mm },
        { "base_dia_mm",       in.base_dia_mm },
        { "barb_pitch_mm",     in.barb_pitch_mm },
        { "derived_volume_added_mm3", vAdded },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "lathe;form_turn";
    tooling.tool_dia_mm       = in.barb_max_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.06;
    tooling.stock_removed_mm3 = -vAdded;   // negative — material added
    tooling.est_cycle_time_s  = std::max(6.0, in.barb_count * 2.5);
    tooling.extra = {
        { "ridge_count", in.barb_count },
        { "standard",    "SAE J1231 hose barb fitting" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::hose_barb_compound: barbs={} base_dia={}",
                  in.barb_count, in.base_dia_mm);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric: count +Z-aligned cylindrical faces (base body + ridges).
    int axialCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.Z()) > 0.9) ++axialCyls;
        } catch (...) {}
    }
    if (axialCyls >= 2) {
        json recovered = { { "barb_count_estimate", std::max(0, axialCyls - 1) } };
        json matched   = { { "source", "geometric_ridge_cyls" },
                           { "axial_cyl_count", axialCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::hose_barb_compound
