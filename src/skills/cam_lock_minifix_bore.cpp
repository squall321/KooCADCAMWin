// @lat: [[engine/skills#cam_lock_minifix_bore]]

#include "cam_lock_minifix_bore.hpp"

#include "Workpiece.hpp"
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

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::cam_lock_minifix_bore {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!(in.cam_dia_mm > 0.0) || !(in.cam_depth_mm > 0.0) ||
        !(in.dowel_dia_mm > 0.0) || !(in.dowel_offset_mm > 0.0) ||
        !(in.bolt_dia_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "cam_lock_minifix_bore: all dimensions must be > 0");
        return r;
    }
    if (in.cam_dia_mm < 10.0 || in.cam_dia_mm > 25.0) {
        r.add("DFM-CAMDIA", "error",
              "cam_lock_minifix_bore: cam_dia_mm " +
              std::to_string(in.cam_dia_mm) +
              " out of range [10, 25] for a Minifix cam housing");
    }
    // Dowel bore must clear the cam wall (no overlap of the two vertical bores).
    const double clearance =
        in.dowel_offset_mm - (in.cam_dia_mm / 2.0 + in.dowel_dia_mm / 2.0);
    if (clearance <= 0.0) {
        r.add("DFM-DOWELFIT", "error",
              "cam_lock_minifix_bore: dowel_offset too small — dowel bore "
              "overlaps the cam housing bore");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cam_lock_minifix_bore DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    // ── 1) Cam housing bore (blind, vertical) ────────────────────────────
    const double camR = in.cam_dia_mm / 2.0;
    const gp_Pnt camStart(cx, cy, topZ - in.cam_depth_mm);
    const gp_Ax2 camAx(camStart, gp::DZ());
    TopoDS_Shape current =
        pr::cut(wp.shape(), pr::cylinder(camAx, camR, in.cam_depth_mm + kOver));

    // ── 2) Dowel bore (blind, vertical, offset in X, non-overlapping) ────
    const double dowelR = in.dowel_dia_mm / 2.0;
    const double dowelDepth = std::min(in.cam_depth_mm + 2.0, (zMax - zMin));
    const gp_Pnt dowelStart(cx + in.dowel_offset_mm, cy, topZ - dowelDepth);
    const gp_Ax2 dowelAx(dowelStart, gp::DZ());
    current = pr::cut(current, pr::cylinder(dowelAx, dowelR, dowelDepth + kOver));

    // ── 3) Connecting-bolt cross hole (transverse, along +Y into the cam) ─
    // Horizontal cylinder whose axis is +Y, centred on the cam axis at the
    // bolt-engagement depth.  May intersect the cam bore — cut SEQUENTIALLY,
    // last, never as a compound boolean.
    const double boltR = in.bolt_dia_mm / 2.0;
    const double boltZ = topZ - in.cam_depth_mm * 0.5;     // mid-cam engagement
    const double crossLen = (yMax - yMin) + 2.0 * kOver;   // through the panel
    const gp_Pnt boltStart(cx, yMin - kOver, boltZ);
    const gp_Ax2 boltAx(boltStart, gp_Dir(0.0, 1.0, 0.0));
    current = pr::cut(current, pr::cylinder(boltAx, boltR, crossLen));

    const int subfeatures = 3;
    const double vCam   = M_PI * camR * camR * in.cam_depth_mm;
    const double vDowel = M_PI * dowelR * dowelR * dowelDepth;
    const double vBolt  = M_PI * boltR * boltR * (yMax - yMin);
    const double volRemoved = vCam + vDowel + vBolt;

    json params = {
        { "center_xy",       { in.center_xy.X(),
                               in.center_xy.Y(),
                               in.center_xy.Z() } },
        { "cam_dia_mm",      in.cam_dia_mm },
        { "cam_depth_mm",    in.cam_depth_mm },
        { "dowel_dia_mm",    in.dowel_dia_mm },
        { "dowel_offset_mm", in.dowel_offset_mm },
        { "bolt_dia_mm",     in.bolt_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "furniture_feature_type",     "cam_lock_minifix_joint" },
        { "subfeature_count",           subfeatures },
        { "derived_cam_dia_mm",         in.cam_dia_mm },
        { "derived_cam_depth_mm",       in.cam_depth_mm },
        { "derived_dowel_dia_mm",       in.dowel_dia_mm },
        { "derived_dowel_offset_mm",    in.dowel_offset_mm },
        { "derived_bolt_dia_mm",        in.bolt_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "Minifix knock-down" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "forstner;drill";
    tooling.tool_dia_mm       = in.cam_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 160.0;
    tooling.feed_per_tooth_mm = 0.07;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(18.0, in.cam_depth_mm * 1.4 + 12.0);
    tooling.extra = {
        { "furniture_application", "knock_down_cam_lock" },
        { "standard",              "Minifix knock-down" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::cam_lock_minifix_bore: cam_dia={} dowel_off={} bolt={}",
                  in.cam_dia_mm, in.dowel_offset_mm, in.bolt_dia_mm);

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

    // Geometric: one mid cam cylinder (~7.5 mm radius), a dowel cylinder and
    // a transverse bolt cylinder.
    int camCyls = 0;
    int smallCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 5.0 && radius <= 12.5) ++camCyls;
            else if (radius >= 2.0 && radius < 5.0) ++smallCyls;
        } catch (...) {}
    }
    if (camCyls >= 1 && smallCyls >= 1) {
        json recovered = { { "cam_dia_mm",      15.0 },
                           { "cam_depth_mm",    12.5 },
                           { "dowel_dia_mm",    8.0 },
                           { "dowel_offset_mm", 32.0 },
                           { "bolt_dia_mm",     7.0 } };
        json matched   = { { "source",     "geometric_cam_dowel_bolt_pattern" },
                           { "cam_cyls",   camCyls },
                           { "small_cyls", smallCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::cam_lock_minifix_bore
