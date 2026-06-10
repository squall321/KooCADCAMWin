// @lat: [[engine/skills#keyway_din6885_parallel_shaft]]

#include "keyway_din6885_parallel_shaft.hpp"

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

namespace koocadcam::skill::keyway_din6885_parallel_shaft {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

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
    const keyway::Din6885ParallelBand* band =
        keyway::findDin6885Band(in.shaft_dia_mm);
    if (band == nullptr) {
        r.add("DFM-DIN6885-RANGE", "error",
              "shaft_dia_mm " + std::to_string(in.shaft_dia_mm) +
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

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "keyway_din6885_parallel_shaft DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const keyway::Din6885ParallelBand* band =
        keyway::findDin6885Band(in.shaft_dia_mm);
    // validate() already guarded this; defensive nullptr check kept for /WX.
    if (band == nullptr)
        throw SkillError("keyway_din6885_parallel_shaft: band lookup failed");

    auto faceId = wp.resolve(in.face_id);
    // face datum is optional (FaceByNormal may fail on a curved shaft); we
    // proceed with shaft_axis as the truth.
    const int faceIdResolved = faceId.value_or(-1);

    const double cA = std::cos(in.angle_rad);
    const double sA = std::sin(in.angle_rad);
    const gp_Dir radialOut(cA, sA, 0.0);
    const gp_Dir tangentCCW(-sA, cA, 0.0);

    const double shaftR = in.shaft_dia_mm / 2.0;
    const double width  = band->key_width_mm;
    const double depth  = band->shaft_keyway_depth_mm;
    const double length = in.key_length_mm;

    // Surface point on the shaft OD.
    const gp_Pnt surfacePt(
        in.shaft_center_x_mm + shaftR * radialOut.X(),
        in.shaft_center_y_mm + shaftR * radialOut.Y(),
        in.key_position_z_mm);

    // Cutter origin: surface_pt − (width/2)·tangent − (length/2)·shaft_axis
    // − depth · radialOut (push inward).
    const gp_Pnt origin(
        surfacePt.X() - width / 2.0 * tangentCCW.X()
                      - length / 2.0 * in.shaft_axis.X()
                      - depth        * radialOut.X(),
        surfacePt.Y() - width / 2.0 * tangentCCW.Y()
                      - length / 2.0 * in.shaft_axis.Y()
                      - depth        * radialOut.Y(),
        surfacePt.Z() - width / 2.0 * tangentCCW.Z()
                      - length / 2.0 * in.shaft_axis.Z()
                      - depth        * radialOut.Z());

    // gp_Ax2(origin, radialOut, shaft_axis):
    //   DX along shaft_axis (length), DY along tangentCCW (width),
    //   DZ along radialOut (2·depth so the box overshoots the OD).
    const gp_Ax2 boxAx(origin, radialOut, in.shaft_axis);
    const TopoDS_Shape cutter = pr::box(boxAx, length, width, 2.0 * depth);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double volume_removed_mm3 = length * width * depth;

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
        { "keyway_standard",       "DIN_6885_A_parallel_shaft" },
        { "key_width_mm",          width },
        { "key_height_mm",         band->key_height_mm },
        { "key_depth_mm",          depth },
        { "tolerance_grade",       band->tolerance_grade },
        { "derived_volume_removed", volume_removed_mm3 },
        { "end_geometry",          "round" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = width;
    tooling.tool_length_mm    = depth * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volume_removed_mm3;
    tooling.est_cycle_time_s  = std::max(2.0, length * 0.05);
    tooling.extra = {
        { "standard",         "DIN 6885 Part A (parallel key, shaft seat)" },
        { "tolerance_grade",  band->tolerance_grade },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::keyway_din6885_parallel_shaft applied: dia={} L={} b={} t1={}",
                  in.shaft_dia_mm, length, width, depth);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric recognizer: a rectangular pocket on a cylindrical shaft face
// produces the same 4-wall + 1-floor signature as `keyway_external`.  The
// reliable identifier is the metadata replay path; we additionally detect
// a remaining cylindrical face of original shaft radius alongside a 4-wall
// planar pocket, then re-match width/depth to the DIN bands.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay.
    for (const auto& feat : wp.features()) {
        if (feat.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, feat.params, 1.0,
            { { "source", "metadata_replay" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback: look for a cylindrical OD face + adjacent planar
    // pocket walls.  Recover band by matching the dominant cylinder radius.
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface as(wp.face(i));
        const gp_Cylinder c = as.Cylinder();
        const double dia = 2.0 * c.Radius();
        const auto* band = keyway::findDin6885Band(dia);
        if (band == nullptr) continue;
        // Heuristic: require at least 4 planar faces (the four pocket walls).
        int planarCount = 0;
        for (int j = 0; j < wp.faceCount(); ++j)
            if (wp.isFacePlanar(j)) ++planarCount;
        if (planarCount < 4) continue;
        json recovered = {
            { "shaft_dia_mm", dia },
            { "key_width_mm", band->key_width_mm },
            { "shaft_keyway_depth_mm", band->shaft_keyway_depth_mm },
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

}  // namespace koocadcam::skill::keyway_din6885_parallel_shaft
