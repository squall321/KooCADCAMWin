// @lat: [[engine/skills#drum_lug_seat_compound]]

#include "drum_lug_seat_compound.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::drum_lug_seat_compound {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

// Default UNC thread per lug size.  Drum lugs traditionally use UNC threads;
// these defaults map to the closest standard size that exists in our central
// UNC/UNF table (_iso_thread_table.hpp): "#8-32" is the canonical small-tom
// lug; "1/4-20" is the canonical large/kick lug.
std::string defaultThreadFor(const std::string& lug_size)
{
    if (lug_size == "small") return "#8-32";
    if (lug_size == "large") return "1/4-20";
    return "";
}

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.tap_depth_mm <= 0.0 || in.insert_recess_dia_mm <= 0.0
        || in.insert_recess_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "drum_lug_seat_compound: positive dimensions required");
    }

    if (in.lug_size != "small" && in.lug_size != "large") {
        r.add("DFM-LUG-SIZE", "error",
              "drum_lug_seat_compound: lug_size '" + in.lug_size
              + "' must be 'small' or 'large'");
    }

    // Resolve thread key: explicit override OR default-by-lug-size.
    const std::string key = in.thread_size_key.empty()
        ? defaultThreadFor(in.lug_size)
        : in.thread_size_key;
    if (key.empty() || tt::findUncUnf(key) == nullptr) {
        r.add("DFM-THREAD-KEY", "error",
              "drum_lug_seat_compound: unknown UNC thread_size_key '"
              + key + "' (must exist in central UNC/UNF table)");
        (void)wp;
        return r;
    }

    const auto* spec = tt::findUncUnf(key);
    if (spec && in.insert_recess_dia_mm < spec->tap_pilot_dia_mm + 2.0) {
        r.add("DFM-INSERT-DIA", "error",
              "drum_lug_seat_compound: insert_recess_dia "
              + std::to_string(in.insert_recess_dia_mm)
              + " mm must exceed pilot dia + 2 mm ("
              + std::to_string(spec->tap_pilot_dia_mm + 2.0) + " mm)");
    }
    if (in.insert_recess_depth_mm >= in.tap_depth_mm) {
        r.add("DFM-INSERT-DEPTH", "error",
              "drum_lug_seat_compound: insert_recess_depth must be < tap_depth");
    }
    if (!(std::abs(in.axis_dir.X()) < 1e-6 && std::abs(in.axis_dir.Y()) < 1e-6)) {
        r.add("DFM-AXIS", "error",
              "drum_lug_seat_compound: only Z-aligned axis_dir is supported");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "drum_lug_seat_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string key = in.thread_size_key.empty()
        ? defaultThreadFor(in.lug_size)
        : in.thread_size_key;
    const auto* spec = tt::findUncUnf(key);
    if (!spec) throw SkillError(
        "drum_lug_seat_compound: thread lookup failed for '" + key + "'");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    constexpr double kOver = 0.1;
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;

    // Entry plane: if axis_dir points down (-Z), enter from zMax; if +Z, enter
    // from zMin.
    const bool fromTop = in.axis_dir.Z() < 0.0;
    const double entryZ = fromTop ? zMax : zMin;
    const gp_Dir axisDir = in.axis_dir;

    const double pilotR  = spec->tap_pilot_dia_mm / 2.0;
    const double insertR = in.insert_recess_dia_mm / 2.0;

    // ── 1) Tap pilot hole — cylinder cut, depth = tap_depth_mm + overhang ──
    const gp_Pnt pilotOrigin(cx, cy, entryZ + (fromTop ? kOver : -kOver));
    const gp_Ax2 pilotAx(pilotOrigin, axisDir);
    const TopoDS_Shape pilotTool = pr::cylinder(pilotAx, pilotR,
        in.tap_depth_mm + 2.0 * kOver);
    TopoDS_Shape current = pr::cut(wp.shape(), pilotTool);

    // ── 2) Counterbore for threaded insert — wider cylinder, shallower depth.
    const gp_Pnt insertOrigin(cx, cy, entryZ + (fromTop ? kOver : -kOver));
    const gp_Ax2 insertAx(insertOrigin, axisDir);
    const TopoDS_Shape insertTool = pr::cylinder(insertAx, insertR,
        in.insert_recess_depth_mm + 2.0 * kOver);
    current = pr::cut(current, insertTool);

    const double vPilot  = M_PI * pilotR  * pilotR  * in.tap_depth_mm;
    const double vInsert = M_PI * (insertR * insertR - pilotR * pilotR)
                         * in.insert_recess_depth_mm;
    const double volRemoved = vPilot + std::max(0.0, vInsert);

    json params = {
        { "center_x_mm",            cx },
        { "center_y_mm",            cy },
        { "axis_dir",               { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "lug_size",               in.lug_size },
        { "thread_size_key",        key },
        { "tap_depth_mm",           in.tap_depth_mm },
        { "insert_recess_dia_mm",   in.insert_recess_dia_mm },
        { "insert_recess_depth_mm", in.insert_recess_depth_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "audio_feature_type",  "drum_lug_seat" },
        { "subfeature_count",    2 },
        { "lug_size",            in.lug_size },
        { "thread_size_key",     key },
        { "pilot_dia_mm",        spec->tap_pilot_dia_mm },
        { "thread_nominal_dia_mm", spec->nominal_dia_mm },
        { "insert_recess_dia_mm", in.insert_recess_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;tap;counterbore";
    tooling.tool_dia_mm       = in.insert_recess_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(8.0, in.tap_depth_mm * 0.4 + 4.0);
    tooling.extra = {
        { "standard",         "drum hardware tension lug seat" },
        { "thread_table",     "ASME B1.1 UNC (central UNC/UNF table)" },
        { "drum_size_class",  in.lug_size },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::drum_lug_seat_compound: thread={} insert_dia={}",
                  key, in.insert_recess_dia_mm);

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

    // Geometric: two coaxial cylinders with different radii — wide one shallow,
    // narrow one deep (counterbore + pilot pattern).
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
        json recovered = { { "lug_pattern", "coaxial_pilot_counterbore" } };
        json matched   = { { "source", "geometric_drum_lug_pattern" },
                           { "axial_cyl_count", axialCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::drum_lug_seat_compound
