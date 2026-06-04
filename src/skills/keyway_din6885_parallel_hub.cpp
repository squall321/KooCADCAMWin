// @lat: [[engine/skills#keyway_din6885_parallel_hub]]

#include "keyway_din6885_parallel_hub.hpp"

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

namespace koocadcam::skill::keyway_din6885_parallel_hub {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.bore_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bore_dia_mm must be > 0");
        return r;
    }
    if (in.key_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "key_length_mm must be > 0");
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
    if (in.key_length_mm < 1.5 * band->key_width_mm) {
        r.add("DFM-DIN6885-LEN", "error",
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
        std::string msg = "keyway_din6885_parallel_hub DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }
    const keyway::Din6885ParallelBand* band =
        keyway::findDin6885Band(in.bore_dia_mm);
    if (band == nullptr)
        throw SkillError("keyway_din6885_parallel_hub: band lookup failed");

    auto faceId = wp.resolve(in.bore_face_id);
    const int faceIdResolved = faceId.value_or(-1);

    const double cA = std::cos(in.angle_rad);
    const double sA = std::sin(in.angle_rad);
    const gp_Dir radialOut(cA, sA, 0.0);
    const gp_Dir tangentCCW(-sA, cA, 0.0);

    const double boreR = in.bore_dia_mm / 2.0;
    const double width = band->key_width_mm;
    const double depth = band->hub_keyway_depth_mm;
    const double length = in.key_length_mm;

    // Surface point on the BORE wall (inward-facing cylinder).
    // We want to cut OUTWARD from the bore (away from center) into the hub.
    const gp_Pnt borePt(
        in.bore_center_x_mm + boreR * radialOut.X(),
        in.bore_center_y_mm + boreR * radialOut.Y(),
        in.key_position_z_mm);

    // Box origin: bore surface − width/2 along tangent − length/2 along axis.
    // The cutter extends OUTWARD (+radialOut) by depth (square-end broach
    // path).  Push the box slightly into the bore (-1.0 along radialOut)
    // to ensure the Boolean produces a clean intersection with the bore
    // wall and isn't degenerate at the cylindrical face.
    const double bite = 1.0;
    const gp_Pnt origin(
        borePt.X() - width  / 2.0 * tangentCCW.X()
                   - length / 2.0 * in.bore_axis.X()
                   - bite          * radialOut.X(),
        borePt.Y() - width  / 2.0 * tangentCCW.Y()
                   - length / 2.0 * in.bore_axis.Y()
                   - bite          * radialOut.Y(),
        borePt.Z() - width  / 2.0 * tangentCCW.Z()
                   - length / 2.0 * in.bore_axis.Z()
                   - bite          * radialOut.Z());

    // gp_Ax2(origin, radialOut, bore_axis):
    //   DX along bore_axis (length), DY along tangentCCW (width),
    //   DZ along radialOut (depth + bite — broaches outward into hub).
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
        { "key_position_z_mm", in.key_position_z_mm },
        { "key_length_mm",    in.key_length_mm },
        { "angle_rad",        in.angle_rad },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "keyway_standard",       "DIN_6885_A_parallel_hub" },
        { "key_width_mm",          width },
        { "key_height_mm",         band->key_height_mm },
        { "key_depth_mm",          depth },
        { "tolerance_grade",       band->tolerance_grade },
        { "derived_volume_removed", volume_removed_mm3 },
        { "end_geometry",          "square" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "broach";
    tooling.tool_dia_mm       = width;
    tooling.tool_length_mm    = length + 20.0;   // broach is long
    tooling.tool_material     = "hss";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 30.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volume_removed_mm3;
    tooling.est_cycle_time_s  = std::max(3.0, length * 0.2 + depth * 1.0);
    tooling.extra = {
        { "standard",        "DIN 6885 Part A (parallel key, hub seat)" },
        { "tolerance_grade", band->tolerance_grade },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::keyway_din6885_parallel_hub applied: bore={} L={} b={} t2={}",
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

    // Geometric fallback: find a bore (cylindrical) face whose dia matches
    // a DIN band, and report it as a candidate (the exact slot geometry
    // requires a much fuller topo walk than is in scope here).
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
            { "bore_dia_mm",       dia },
            { "key_width_mm",      band->key_width_mm },
            { "hub_keyway_depth_mm", band->hub_keyway_depth_mm },
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

}  // namespace koocadcam::skill::keyway_din6885_parallel_hub
