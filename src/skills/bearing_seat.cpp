// @lat: [[engine/skills#bearing_seat]]

#include "bearing_seat.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::bearing_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.outer_dia_mm < 4.0) {
        r.add("DFM-BS-DIA", "error",
              "bearing_seat outer_dia_mm " + std::to_string(in.outer_dia_mm) +
              " < 4 mm");
    }
    if (in.bore_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bore_depth_mm must be > 0");
    }
    static const char* okFits[] = { "k5","k6","m6","n6","p6","H7" };
    bool fitOk = false;
    for (const char* f : okFits)
        if (in.fit_class == f) { fitOk = true; break; }
    if (!fitOk) {
        r.add("DFM-BS-FIT", "warning",
              "unknown fit_class '" + in.fit_class + "' — expected k5/k6/m6/n6/p6/H7");
    }
    if (in.preload_N < 0.0) {
        r.add("DFM-INPUT", "warning", "preload_N negative — interpreting as 0");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bearing_seat DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("bearing_seat: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    // Cut a cylinder of (outer_dia_mm / 2) radius, depth = bore_depth_mm, into
    // the entry face.  The bottom of this cylinder is the shoulder face.
    const gp_Dir adir = in.axis_dir;
    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0)
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
        else
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kOverhang);
    } else {
        toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, (zMin + zMax) / 2.0);
    }

    const double toolHeight = in.bore_depth_mm + kOverhang;
    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, in.outer_dia_mm / 2.0, toolHeight);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "outer_dia_mm",    in.outer_dia_mm },
        { "bore_depth_mm",   in.bore_depth_mm },
        { "bearing_id",      in.bearing_id },
        { "fit_class",       in.fit_class },
        { "preload_N",       in.preload_N },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "shoulder_planar_face",       true },
        { "outer_dia_mm",               in.outer_dia_mm },
        { "bore_depth_mm",              in.bore_depth_mm },
        { "bearing_id",                 in.bearing_id },
        { "fit_class",                  in.fit_class },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar";
    tooling.tool_dia_mm       = in.outer_dia_mm * 0.75;
    tooling.tool_length_mm    = in.bore_depth_mm * 1.5 + 8.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 150.0;
    tooling.feed_per_tooth_mm = 0.02;
    tooling.stock_removed_mm3 = M_PI * (in.outer_dia_mm / 2.0) * (in.outer_dia_mm / 2.0)
                              * in.bore_depth_mm;
    tooling.est_cycle_time_s  = std::max(3.0, in.bore_depth_mm * 1.2);
    tooling.extra = {
        { "fit_class",  in.fit_class },
        { "preload_N",  in.preload_N },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::bearing_seat applied: OD={} depth={} fit={} faces {}→{}",
                  in.outer_dia_mm, in.bore_depth_mm, in.fit_class,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// Metadata replay: bearing_id / fit_class / preload cannot be recovered from
// geometry alone; rely on feature history.
std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 0.85,
            { { "via", "metadata_replay" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::bearing_seat
