// @lat: [[engine/skills#keyway_internal_din6885_through]]

#include "keyway_internal_din6885_through.hpp"

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

namespace koocadcam::skill::keyway_internal_din6885_through {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.bore_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bore_dia_mm must be > 0");
        return r;
    }
    const keyway::Din6885ParallelBand* band =
        keyway::findDin6885Band(in.bore_dia_mm);
    if (band == nullptr) {
        r.add("DFM-DIN6885-RANGE", "error",
              "bore_dia_mm " + std::to_string(in.bore_dia_mm) +
              " not in DIN 6885 bands (6..95 mm)");
        return r;
    }
    // The "key_length > 1.5 × width" rule still applies — for through cuts we
    // compute length from the workpiece bbox.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxLen = zMax - zMin;
    if (bboxLen < 1.5 * band->key_width_mm) {
        r.add("DFM-DIN6885-LEN", "error",
              "workpiece axial extent " + std::to_string(bboxLen) +
              " < 1.5 * key_width " + std::to_string(band->key_width_mm));
    }
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "keyway_internal_din6885_through DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }
    const keyway::Din6885ParallelBand* band =
        keyway::findDin6885Band(in.bore_dia_mm);
    if (band == nullptr)
        throw SkillError("keyway_internal_din6885_through: band lookup failed");

    auto faceId = wp.resolve(in.bore_face_id);
    const int faceIdResolved = faceId.value_or(-1);

    const double cA = std::cos(in.angle_rad);
    const double sA = std::sin(in.angle_rad);
    const gp_Dir radialOut(cA, sA, 0.0);
    const gp_Dir tangentCCW(-sA, cA, 0.0);

    const double boreR = in.bore_dia_mm / 2.0;
    const double width = band->key_width_mm;
    const double depth = band->hub_keyway_depth_mm;

    // Through-cut: extend cutter the full workpiece Z-extent + overhang.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double overhang = 0.5;
    const double length = (zMax - zMin) + 2.0 * overhang;
    const double bite = 1.0;

    // Bore surface point at lowest Z (the cutter origin).
    const gp_Pnt borePt(
        in.bore_center_x_mm + boreR * radialOut.X(),
        in.bore_center_y_mm + boreR * radialOut.Y(),
        zMin - overhang);

    const gp_Pnt origin(
        borePt.X() - width / 2.0 * tangentCCW.X()
                   - bite        * radialOut.X(),
        borePt.Y() - width / 2.0 * tangentCCW.Y()
                   - bite        * radialOut.Y(),
        borePt.Z() - width / 2.0 * tangentCCW.Z()
                   - bite        * radialOut.Z());

    const gp_Ax2 boxAx(origin, radialOut, in.bore_axis);
    const TopoDS_Shape cutter = pr::box(boxAx, length, width, depth + bite);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double volume_removed_mm3 = length * width * depth;

    json params = {
        { "face_id_resolved", faceIdResolved },
        { "bore_axis",        { in.bore_axis.X(), in.bore_axis.Y(), in.bore_axis.Z() } },
        { "bore_center_x_mm", in.bore_center_x_mm },
        { "bore_center_y_mm", in.bore_center_y_mm },
        { "bore_dia_mm",      in.bore_dia_mm },
        { "angle_rad",        in.angle_rad },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "keyway_standard",       "DIN_6885_A_internal_through" },
        { "key_width_mm",          width },
        { "key_height_mm",         band->key_height_mm },
        { "key_depth_mm",          depth },
        { "tolerance_grade",       band->tolerance_grade },
        { "derived_volume_removed", volume_removed_mm3 },
        { "end_geometry",          "through" },
        { "axial_extent_mm",       length },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "broach";
    tooling.tool_dia_mm       = width;
    tooling.tool_length_mm    = length + 30.0;
    tooling.tool_material     = "hss";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 25.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volume_removed_mm3;
    tooling.est_cycle_time_s  = std::max(4.0, length * 0.25);
    tooling.extra = {
        { "standard",        "DIN 6885 Part A (parallel key, hub seat, through)" },
        { "tolerance_grade", band->tolerance_grade },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::keyway_internal_din6885_through applied: bore={} L={} b={} t2={}",
                  in.bore_dia_mm, length, width, depth);

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

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface as(wp.face(i));
        const gp_Cylinder c = as.Cylinder();
        const double dia = 2.0 * c.Radius();
        const auto* band = keyway::findDin6885Band(dia);
        if (band == nullptr) continue;
        int planarCount = 0;
        for (int j = 0; j < wp.faceCount(); ++j)
            if (wp.isFacePlanar(j)) ++planarCount;
        if (planarCount < 4) continue;
        json recovered = {
            { "bore_dia_mm",          dia },
            { "key_width_mm",         band->key_width_mm },
            { "hub_keyway_depth_mm",  band->hub_keyway_depth_mm },
        };
        json matched = {
            { "source",            "geometric_fallback" },
            { "cyl_face_id",       i },
            { "planar_face_count", planarCount },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.50, matched });
        break;
    }
    return out;
}

}  // namespace koocadcam::skill::keyway_internal_din6885_through
