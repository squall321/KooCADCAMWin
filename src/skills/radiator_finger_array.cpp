// @lat: [[engine/skills#radiator_finger_array]]

#include "radiator_finger_array.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::radiator_finger_array {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.fin_count < 2) {
        r.add("DFM-FIN-N", "error",
              "radiator_finger_array: fin_count must be >= 2");
    }
    if (in.fin_thickness_mm < 0.15) {
        r.add("DFM-FIN-THK", "error",
              "radiator_finger_array: fin_thickness " +
              std::to_string(in.fin_thickness_mm) +
              " mm < 0.15 mm (stamping / cutter floor)");
    }
    if (in.fin_pitch_mm <= in.fin_thickness_mm) {
        r.add("DFM-FIN-PITCH", "error",
              "radiator_finger_array: fin_pitch must exceed fin_thickness");
    }
    if (in.fin_length_mm <= 0.0 || in.fin_height_mm <= 0.0 ||
        in.tube_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "radiator_finger_array: positive dimensions required");
    }
    if (in.tube_dia_mm >= in.fin_height_mm) {
        r.add("DFM-TUBE-FIT", "error",
              "radiator_finger_array: tube_dia >= fin_height (tube cannot fit)");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "radiator_finger_array DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double midZ = (zMin + zMax) / 2.0;
    const double tubeR = in.tube_dia_mm / 2.0;
    const double centreY = (yMin + yMax) / 2.0;

    // Tubes run along +X through the full stock length.  Two tubes spaced
    // ±fin_height/3 in Y.
    const double tubeOffsetY = std::max(in.fin_height_mm / 3.0,
                                        in.tube_dia_mm + 1.0);
    const double y1 = centreY - tubeOffsetY * 0.5;
    const double y2 = centreY + tubeOffsetY * 0.5;

    const gp_Pnt t1Start(xMin - kOver, y1, midZ);
    const gp_Pnt t2Start(xMin - kOver, y2, midZ);
    const double tubeLen = (xMax - xMin) + 2.0 * kOver;

    // ── 1) inlet tube bore (cut #1) ──────────────────────────────────────
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::cylinder(gp_Ax2(t1Start, gp::DX()), tubeR, tubeLen));

    // ── 2) outlet tube bore (cut #2) ─────────────────────────────────────
    current = pr::cut(
        current,
        pr::cylinder(gp_Ax2(t2Start, gp::DX()), tubeR, tubeLen));

    // ── 3..) N-1 air-gap fin cuts (subtractive between fins) ─────────────
    //
    // Place N fins of thickness `fin_thickness_mm` evenly across the stock
    // length along +X.  The cuts that REMOVE material are the (N-1) gaps
    // between fins (and the end air strips, but we keep the model compact).
    //
    // Each gap box: width X = (pitch − thickness), span Y = full bbox Y,
    // span Z = full bbox Z.  Located at x = xMin + i*pitch + thickness/2.
    const int gapCount = in.fin_count - 1;
    const double gapWidth = in.fin_pitch_mm - in.fin_thickness_mm;
    const double startX = xMin
                          + ((xMax - xMin) - (in.fin_count - 1) * in.fin_pitch_mm) / 2.0;

    for (int i = 0; i < gapCount; ++i) {
        const double gapX = startX + in.fin_thickness_mm * 0.5
                            + i * in.fin_pitch_mm;
        const gp_Pnt origin(gapX, yMin - kOver, zMin - kOver);
        const gp_Ax2 ax(origin, gp::DZ());
        const TopoDS_Shape gapTool = pr::box(
            ax,
            gapWidth,
            (yMax - yMin) + 2.0 * kOver,
            (zMax - zMin) + 2.0 * kOver);
        current = pr::cut(current, gapTool);
    }

    const double tubeVol = 2.0 * M_PI * tubeR * tubeR * (xMax - xMin);
    const double gapVol  = gapCount * gapWidth * (yMax - yMin) * (zMax - zMin);
    const double volRemoved = tubeVol + gapVol;

    json params = {
        { "fin_count",         in.fin_count },
        { "fin_pitch_mm",      in.fin_pitch_mm },
        { "fin_thickness_mm",  in.fin_thickness_mm },
        { "fin_length_mm",     in.fin_length_mm },
        { "fin_height_mm",     in.fin_height_mm },
        { "tube_dia_mm",       in.tube_dia_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "hvac_feature_type",   "radiator_finger_array" },
        { "subfeature_count",    2 + gapCount },
        { "fin_count",           in.fin_count },
        { "tube_count",          2 },
        { "fin_pitch_mm",        in.fin_pitch_mm },
        { "fin_thickness_mm",    in.fin_thickness_mm },
        { "tube_dia_mm",         in.tube_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;slot_mill";
    tooling.tool_dia_mm       = in.tube_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(10.0, 4.0 + in.fin_count * 1.5);
    tooling.extra = {
        { "tube_count",  2 },
        { "gap_count",   gapCount },
        { "standard",    "ASHRAE finger-fin tube radiator" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::radiator_finger_array: fins={} pitch={} tube_dia={}",
                  in.fin_count, in.fin_pitch_mm, in.tube_dia_mm);

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

    // Geometric fallback: count +X-aligned cylindrical faces (tubes).
    int tubeCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.X()) > 0.9) ++tubeCyls;
        } catch (...) {}
    }
    if (tubeCyls >= 2) {
        json recovered = { { "tube_count", tubeCyls } };
        json matched   = { { "source", "geometric_tube_cyls" },
                           { "tube_cyl_count", tubeCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::radiator_finger_array
