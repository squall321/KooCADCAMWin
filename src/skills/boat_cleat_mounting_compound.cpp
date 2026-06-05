// @lat: [[engine/skills#boat_cleat_mounting_compound]]

#include "boat_cleat_mounting_compound.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
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

namespace koocadcam::skill::boat_cleat_mounting_compound {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cleat_length_mm <= 0.0 || in.bolt_spacing_mm <= 0.0 ||
        in.pad_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "boat_cleat_mounting_compound: all dimensions must be > 0");
    }

    if (in.bolt_thread_size_key.empty()) {
        r.add("DFM-M-THREAD", "error",
              "boat_cleat_mounting_compound: bolt_thread_size_key is empty");
    } else if (!tt::findMetric(in.bolt_thread_size_key)) {
        r.add("DFM-M-THREAD", "error",
              "boat_cleat_mounting_compound: bolt_thread_size_key '" +
              in.bolt_thread_size_key +
              "' not present in central _iso_thread_table.hpp");
    }

    if (in.bolt_spacing_mm >= in.cleat_length_mm) {
        r.add("DFM-MARINE-SPAN", "error",
              "boat_cleat_mounting_compound: bolt_spacing_mm (" +
              std::to_string(in.bolt_spacing_mm) +
              ") must be < cleat_length_mm (" +
              std::to_string(in.cleat_length_mm) + ")");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "boat_cleat_mounting_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const tt::MetricThreadSpec* mSpec = tt::findMetric(in.bolt_thread_size_key);
    if (!mSpec) throw SkillError("boat_cleat_mounting_compound: thread lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    const double boltClr = mSpec->clearance_medium_mm;

    // ── 1) Recessed mounting pad (rectangular box cut) ─────────────────
    // Pad footprint: length along X = cleat_length, width along Y sized to
    // comfortably hold the base (cleat_length * 0.45, marine practice).
    const double padLen   = in.cleat_length_mm;
    const double padWid   = std::max(in.cleat_length_mm * 0.45, in.bolt_spacing_mm * 0.6);
    const double padDepth = in.pad_depth_mm;
    const gp_Pnt padOrigin(cx - padLen / 2.0, cy - padWid / 2.0, topZ - padDepth);
    const gp_Ax2 padAx(padOrigin, gp::DZ());
    const TopoDS_Shape padTool = pr::box(padAx, padLen, padWid, padDepth + kOver);
    TopoDS_Shape current = pr::cut(wp.shape(), padTool);

    // ── 2) & 3) Two bolt clearance holes on bolt_spacing (along X) ─────
    // The bolts pass fully through the plate (deck/backing plate).
    const double thru = (zMax - zMin) + 2.0 * kOver;
    const double holeR = boltClr / 2.0;
    const double half = in.bolt_spacing_mm / 2.0;
    for (int i = 0; i < 2; ++i) {
        const double bx = cx + (i == 0 ? -half : +half);
        const gp_Ax2 holeAx(gp_Pnt(bx, cy, zMin - kOver), gp::DZ());
        const TopoDS_Shape holeTool = pr::cylinder(holeAx, holeR, thru);
        current = pr::cut(current, holeTool);
    }

    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ──────────────────────────────────────
    const double thruPlate = (zMax - zMin);
    const double vPad   = padLen * padWid * padDepth;
    const double vHoles = 2.0 * M_PI * holeR * holeR * thruPlate;
    const double volRemoved = vPad + vHoles;

    json params = {
        { "center_xy",            { in.center_xy.X(),
                                    in.center_xy.Y(),
                                    in.center_xy.Z() } },
        { "cleat_length_mm",      in.cleat_length_mm },
        { "bolt_spacing_mm",      in.bolt_spacing_mm },
        { "bolt_thread_size_key", in.bolt_thread_size_key },
        { "pad_depth_mm",         in.pad_depth_mm },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "marine_feature_type",        "horn_cleat_mounting_base" },
        { "subfeature_count",           3 },
        { "cleat_length_mm",            in.cleat_length_mm },
        { "bolt_spacing_mm",            in.bolt_spacing_mm },
        { "bolt_thread_size",           in.bolt_thread_size_key },
        { "derived_pad_len_mm",         padLen },
        { "derived_pad_wid_mm",         padWid },
        { "derived_pad_depth_mm",       padDepth },
        { "derived_bolt_hole_dia_mm",   boltClr },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "ISO 273 clearance + marine deck practice" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill";
    tooling.tool_dia_mm       = std::max(boltClr, padWid);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(40.0, 25.0 + padLen * 0.4);
    tooling.extra = {
        { "marine_application", "horn_cleat_mounting" },
        { "removed_volume_mm3", volRemoved },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::boat_cleat_mounting_compound: len={} spacing={} thread={} faces {}→{}",
                  in.cleat_length_mm, in.bolt_spacing_mm,
                  in.bolt_thread_size_key, wp.faceCount(), wpNew->faceCount());

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

    // Geometric fallback: a pair of small, equal-radius, +Z-parallel
    // cylindrical bolt holes is the cleat signature.
    std::vector<double> boltRadii;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            if (std::abs(c.Axis().Direction().Z()) < 0.9) continue;
            const double radius = c.Radius();
            if (radius >= 1.5 && radius <= 8.0) boltRadii.push_back(radius);
        } catch (...) {}
    }
    if (boltRadii.size() >= 2) {
        json recovered = { { "bolt_thread_size_key", "M8" },
                           { "cleat_length_mm",      120.0 } };
        json matched   = { { "source",          "geometric_bolt_pair" },
                           { "bolt_hole_count",  static_cast<int>(boltRadii.size()) } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::boat_cleat_mounting_compound
