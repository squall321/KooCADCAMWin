// @lat: [[engine/skills#keyway_din6886_taper_shaft]]

#include "keyway_din6886_taper_shaft.hpp"

#include "_keyway_table.hpp"
#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <string>

namespace koocadcam::skill::keyway_din6886_taper_shaft {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.shaft_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "shaft_dia_mm must be > 0");
        return r;
    }
    if (in.key_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "key_length_mm must be > 0");
        return r;
    }
    const keyway::Din6886TaperBand* band =
        keyway::findDin6886Band(in.shaft_dia_mm);
    if (band == nullptr) {
        r.add("DFM-DIN6886-RANGE", "error",
              "shaft_dia_mm " + std::to_string(in.shaft_dia_mm) +
              " not in DIN 6886 bands (6..95 mm)");
        return r;
    }
    if (in.key_length_mm < 1.5 * band->key_width_mm) {
        r.add("DFM-DIN6886-LEN", "error",
              "key_length_mm " + std::to_string(in.key_length_mm) +
              " < 1.5 * key_width " + std::to_string(band->key_width_mm));
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "keyway_din6886_taper_shaft DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }
    const keyway::Din6886TaperBand* band =
        keyway::findDin6886Band(in.shaft_dia_mm);
    if (band == nullptr)
        throw SkillError("keyway_din6886_taper_shaft: band lookup failed");

    auto faceId = wp.resolve(in.face_id);
    const int faceIdResolved = faceId.value_or(-1);

    const double cA = std::cos(in.angle_rad);
    const double sA = std::sin(in.angle_rad);
    const gp_Dir radialOut(cA, sA, 0.0);
    const gp_Dir tangentCCW(-sA, cA, 0.0);

    const double shaftR  = in.shaft_dia_mm / 2.0;
    const double width   = band->key_width_mm;
    const double depthThick = band->shaft_keyway_depth_mm;            // t1
    const double length  = in.key_length_mm;
    // Taper 1:100: thin-end depth = depthThick - length/100.
    const double depthThin = std::max(0.05, depthThick - length / 100.0);

    // We model the tapered seat as TWO sub-cuts (compound):
    //   1) MAIN box at depth = depthThin (uniform, runs the full length)
    //   2) WEDGE BOX at the thick end, length L/2, depth = depthThick−depthThin
    // This is a discrete two-step approximation; the signature metadata
    // records the analytic 1:100 taper used.

    const gp_Pnt surfacePt(
        in.shaft_center_x_mm + shaftR * radialOut.X(),
        in.shaft_center_y_mm + shaftR * radialOut.Y(),
        in.key_position_z_mm);

    // ── Sub-cut 1: main shallow box ──────────────────────────────────────
    const gp_Pnt origin1(
        surfacePt.X() - width  / 2.0 * tangentCCW.X()
                      - length / 2.0 * in.shaft_axis.X()
                      - depthThin    * radialOut.X(),
        surfacePt.Y() - width  / 2.0 * tangentCCW.Y()
                      - length / 2.0 * in.shaft_axis.Y()
                      - depthThin    * radialOut.Y(),
        surfacePt.Z() - width  / 2.0 * tangentCCW.Z()
                      - length / 2.0 * in.shaft_axis.Z()
                      - depthThin    * radialOut.Z());
    const gp_Ax2 ax1(origin1, radialOut, in.shaft_axis);
    const TopoDS_Shape cutter1 = pr::box(ax1, length, width, 2.0 * depthThin);

    // ── Sub-cut 2: thick-end wedge (lower half-length, extra depth) ──────
    const double wedgeLen = length * 0.5;
    const double wedgeExtra = depthThick - depthThin;
    const gp_Pnt origin2(
        surfacePt.X() - width    / 2.0 * tangentCCW.X()
                      - length   / 2.0 * in.shaft_axis.X()
                      - depthThick     * radialOut.X(),
        surfacePt.Y() - width    / 2.0 * tangentCCW.Y()
                      - length   / 2.0 * in.shaft_axis.Y()
                      - depthThick     * radialOut.Y(),
        surfacePt.Z() - width    / 2.0 * tangentCCW.Z()
                      - length   / 2.0 * in.shaft_axis.Z()
                      - depthThick     * radialOut.Z());
    const gp_Ax2 ax2(origin2, radialOut, in.shaft_axis);
    const TopoDS_Shape cutter2 = pr::box(ax2, wedgeLen, width,
                                          wedgeExtra + 0.1);

    TopoDS_Shape newShape = pr::cut(wp.shape(), cutter1);
    if (wedgeExtra > 1e-6) {
        newShape = pr::cut(newShape, cutter2);
    }

    // Analytical volume: trapezoid prism = ((depthThick + depthThin)/2) * L * b.
    const double volume_removed_mm3 =
        0.5 * (depthThick + depthThin) * length * width;

    json params = {
        { "face_id_resolved", faceIdResolved },
        { "shaft_axis",       { in.shaft_axis.X(), in.shaft_axis.Y(), in.shaft_axis.Z() } },
        { "shaft_center_x_mm", in.shaft_center_x_mm },
        { "shaft_center_y_mm", in.shaft_center_y_mm },
        { "shaft_dia_mm",     in.shaft_dia_mm },
        { "key_position_z_mm", in.key_position_z_mm },
        { "key_length_mm",    in.key_length_mm },
        { "angle_rad",        in.angle_rad },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "keyway_standard",       "DIN_6886_taper_shaft" },
        { "key_width_mm",          width },
        { "key_height_at_thick_end_mm", band->key_thickness_at_thick_end_mm },
        { "key_depth_mm",          depthThick },
        { "key_depth_thin_end_mm", depthThin },
        { "taper_per_100",         band->taper_per_100 },
        { "derived_volume_removed", volume_removed_mm3 },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = width;
    tooling.tool_length_mm    = depthThick * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 380.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volume_removed_mm3;
    tooling.est_cycle_time_s  = std::max(3.0, length * 0.06);
    tooling.extra = {
        { "standard",         "DIN 6886 (taper key, 1:100)" },
        { "taper_per_100",    band->taper_per_100 },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::keyway_din6886_taper_shaft applied: dia={} L={} b={} t1={} taper=1/100",
                  in.shaft_dia_mm, length, width, depthThick);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& feat : wp.features()) {
        if (feat.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, feat.params, 1.0,
            { { "source", "metadata_replay" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback: tapered pocket signature (two cylinder/planar
    // assemblies of differing depths).  Tolerant heuristic: just match on
    // shaft cylinder dia.
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface as(wp.face(i));
        const gp_Cylinder c = as.Cylinder();
        const double dia = 2.0 * c.Radius();
        const auto* band = keyway::findDin6886Band(dia);
        if (band == nullptr) continue;
        json recovered = {
            { "shaft_dia_mm",    dia },
            { "key_width_mm",    band->key_width_mm },
            { "key_depth_mm",    band->shaft_keyway_depth_mm },
            { "taper_per_100",   band->taper_per_100 },
        };
        json matched = {
            { "source",      "geometric_fallback" },
            { "cyl_face_id", i },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.45, matched });
        break;
    }
    return out;
}

}  // namespace koocadcam::skill::keyway_din6886_taper_shaft
