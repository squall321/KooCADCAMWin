// @lat: [[engine/skills#spring_lock_washer_seat_din127]]

#include "spring_lock_washer_seat_din127.hpp"

#include "_retaining_rings.hpp"
#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
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
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::spring_lock_washer_seat_din127 {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;
using retaining_rings::findDin127;
using retaining_rings::Din127Spec;

constexpr double kDiaClearance_mm = 0.4;   // OD clearance over washer OD

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thread_size_key.empty()) {
        r.add("DFM-DIN127", "error",
              "spring_lock_washer_seat_din127: thread_size_key is empty");
        return r;
    }
    const Din127Spec* spec = findDin127(in.thread_size_key);
    if (!spec) {
        r.add("DFM-DIN127", "error",
              "spring_lock_washer_seat_din127: unknown DIN 127 size '" +
              in.thread_size_key + "' (M3..M20 supported)");
        return r;
    }

    if (in.depth_extra_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "spring_lock_washer_seat_din127: depth_extra_mm must be >= 0");
    }

    if (wp.shape().IsNull()) {
        r.add("DFM-INPUT", "error",
              "spring_lock_washer_seat_din127: workpiece is null");
        return r;
    }

    const double seatDia = spec->outer_dia_mm + kDiaClearance_mm;
    const double seatDepth = spec->thickness_mm + in.depth_extra_mm;

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double extentX = bb.xMax - bb.xMin;
    const double extentY = bb.yMax - bb.yMin;
    const double extentZ = bb.zMax - bb.zMin;
    const double minXY = std::min(extentX, extentY);
    if (seatDia > minXY - 1.0) {
        r.add("DFM-DIN127-FIT", "warning",
              "spring_lock_washer_seat_din127: seat Ø " +
              std::to_string(seatDia) +
              " mm leaves <0.5 mm wall on workpiece footprint");
    }
    if (seatDepth >= extentZ - 0.5) {
        r.add("DFM-DIN127-DEPTH", "error",
              "spring_lock_washer_seat_din127: seat depth " +
              std::to_string(seatDepth) +
              " exceeds workpiece thickness " + std::to_string(extentZ));
    }

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) {
        r.add("DFM-INPUT", "error",
              "spring_lock_washer_seat_din127: face_id datum unresolved");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "spring_lock_washer_seat_din127 DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const Din127Spec* spec = findDin127(in.thread_size_key);
    const double seatDia   = spec->outer_dia_mm + kDiaClearance_mm;
    const double seatR     = seatDia / 2.0;
    const double seatDepth = spec->thickness_mm + in.depth_extra_mm;

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const gp_Dir adir = in.axis_dir;
    constexpr double kEntryOverhang = 0.05;
    double entryZ;
    if (adir.Z() < 0) entryZ = bb.zMax + kEntryOverhang;
    else              entryZ = bb.zMin - kEntryOverhang;

    const gp_Ax2 ax(gp_Pnt(in.center_x_mm, in.center_y_mm, entryZ), adir);
    const TopoDS_Shape cutter = pr::cylinder(ax, seatR, seatDepth + kEntryOverhang);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double seatVol = M_PI * seatR * seatR * seatDepth;

    json params = {
        { "face_id_kind",    "datum" },
        { "center_x_mm",     in.center_x_mm },
        { "center_y_mm",     in.center_y_mm },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "thread_size_key", in.thread_size_key },
        { "depth_extra_mm",  in.depth_extra_mm },
    };
    json derived = {
        { "washer_inner_dia_mm",  spec->inner_dia_mm },
        { "washer_outer_dia_mm",  spec->outer_dia_mm },
        { "washer_thickness_mm",  spec->thickness_mm },
        { "washer_gap_height_mm", spec->gap_height_mm },
        { "seat_dia_mm",          seatDia },
        { "seat_depth_mm",        seatDepth },
    };
    json pattern = {
        { "kind",           kSkillId },
        { "is_compound",    true },
        { "ring_standard",  kRingStandard },
        { "ring_size_key",  in.thread_size_key },
        { "derived_dims",   derived },
        { "ring_volume_mm3", seatVol },
        { "axis_dir",       { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = seatDia;
    tooling.tool_length_mm    = seatDepth + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = seatVol;
    tooling.est_cycle_time_s  = std::max(1.5, seatDepth / 6.0);
    tooling.extra = {
        { "standard",        kRingStandard },
        { "thread_size_key", in.thread_size_key },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::spring_lock_washer_seat_din127 applied: thread={} Ø={} d={}",
                  in.thread_size_key, seatDia, seatDepth);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& feat : wp.features()) {
        if (feat.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = feat.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",  "metadata_replay" },
            { "pattern", feat.pattern },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: find a cylindrical pocket whose Ø matches a
    // DIN 127 seat Ø (washer OD + clearance) within ±0.5 mm.
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder c = s.Cylinder();
        const double dia = 2.0 * c.Radius();
        std::string bestKey;
        double bestErr = 1e9;
        for (const auto& sp : retaining_rings::kDin127) {
            const double seatDia = sp.outer_dia_mm + kDiaClearance_mm;
            const double err = std::abs(seatDia - dia);
            if (err < bestErr) { bestErr = err; bestKey = sp.size_key; }
        }
        if (bestKey.empty() || bestErr > 0.5) continue;

        json recovered = {
            { "thread_size_key", bestKey },
            { "center_x_mm",     c.Axis().Location().X() },
            { "center_y_mm",     c.Axis().Location().Y() },
        };
        json matched = {
            { "source",         "geometric_fallback" },
            { "cyl_face_id",    i },
            { "seat_dia_match", dia },
            { "dia_match_err",  bestErr },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::spring_lock_washer_seat_din127
