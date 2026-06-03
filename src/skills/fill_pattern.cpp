// @lat: [[engine/skills#fill_pattern]]

#include "fill_pattern.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::fill_pattern {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Bbox extents of the host face's XY footprint.
struct FaceXY
{
    double xMin, yMin, xMax, yMax;
    double centerX, centerY, centerZ;
    gp_Dir normal;
};

FaceXY faceXYExtents(const Workpiece& wp, int faceId)
{
    FaceXY r{};
    const auto bb = pr::optimalBbox(wp.face(faceId));
    r.xMin = bb.xMin; r.yMin = bb.yMin;
    r.xMax = bb.xMax; r.yMax = bb.yMax;
    const gp_Pnt c = wp.faceCenter(faceId);
    r.centerX = c.X(); r.centerY = c.Y(); r.centerZ = c.Z();
    r.normal = wp.faceNormal(faceId);
    return r;
}

int countGrid(double w, double h, double pitch, double dia)
{
    if (pitch <= 0.0) return 0;
    const int nx = std::max(0, static_cast<int>(std::floor(w / pitch)) + 1);
    const int ny = std::max(0, static_cast<int>(std::floor(h / pitch)) + 1);
    (void)dia;
    return nx * ny;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.source_hole_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "fill_pattern: source_hole_dia_mm must be > 0");
    }
    if (in.source_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "fill_pattern: source_depth_mm must be > 0");
    }
    if (in.source_hole_dia_mm > 0.0 && in.source_hole_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "fill_pattern: source_hole_dia_mm " +
              std::to_string(in.source_hole_dia_mm) +
              " < 0.8 mm (ISO 235:2016)");
    }
    if (in.pitch_mm <= in.source_hole_dia_mm) {
        r.add("DFM-INPUT", "error",
              "fill_pattern: pitch_mm " + std::to_string(in.pitch_mm) +
              " <= source_hole_dia_mm (instances would overlap)");
    }
    if (in.margin_mm < 0.0) {
        r.add("DFM-INPUT", "error", "fill_pattern: margin_mm must be >= 0");
    }
    if (in.fill_type != "grid" && in.fill_type != "hex") {
        r.add("DFM-INPUT", "error",
              "fill_pattern: fill_type must be 'grid' or 'hex' (got '" +
              in.fill_type + "')");
    }
    // DFM-PATT-4 — at least 1 instance must fit.
    if (r.passed) {
        auto faceIdOpt = wp.resolve(in.face);
        if (faceIdOpt) {
            const FaceXY ex = faceXYExtents(wp, *faceIdOpt);
            const double w = (ex.xMax - ex.xMin) - 2.0 * in.margin_mm;
            const double h = (ex.yMax - ex.yMin) - 2.0 * in.margin_mm;
            if (w <= in.source_hole_dia_mm || h <= in.source_hole_dia_mm) {
                r.add("DFM-PATT-4", "error",
                      "fill_pattern: active region (" + std::to_string(w) +
                      " x " + std::to_string(h) + ") cannot fit any hole");
            }
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "fill_pattern DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceIdOpt = wp.resolve(in.face);
    if (!faceIdOpt) throw SkillError("fill_pattern: face datum unresolved");
    const int faceId = *faceIdOpt;
    const FaceXY ex = faceXYExtents(wp, faceId);

    const double xLo = ex.xMin + in.margin_mm;
    const double yLo = ex.yMin + in.margin_mm;
    const double xHi = ex.xMax - in.margin_mm;
    const double yHi = ex.yMax - in.margin_mm;
    if (xHi <= xLo || yHi <= yLo)
        throw SkillError("fill_pattern: margin consumes the whole face");

    const gp_Dir cutDir(-ex.normal.X(), -ex.normal.Y(), -ex.normal.Z());

    constexpr double kOverhang = 0.1;
    const double toolLen = in.source_depth_mm + 2.0 * kOverhang;
    const double r = in.source_hole_dia_mm * 0.5;

    std::vector<TopoDS_Shape> cutters;
    std::vector<std::pair<double,double>> centers;
    if (in.fill_type == "grid") {
        // Walk grid starting at xLo, yLo.
        for (double y = yLo; y <= yHi + 1e-9; y += in.pitch_mm) {
            for (double x = xLo; x <= xHi + 1e-9; x += in.pitch_mm) {
                centers.emplace_back(x, y);
            }
        }
    } else {
        // hex — rows of pitch_mm spacing in Y; alternate rows shifted by
        // pitch_mm/2 in X, with row pitch sqrt(3)/2 * pitch.
        const double rowDy = in.pitch_mm * std::sqrt(3.0) * 0.5;
        int rowIdx = 0;
        for (double y = yLo; y <= yHi + 1e-9; y += rowDy, ++rowIdx) {
            const double xShift = (rowIdx % 2 == 0) ? 0.0 : in.pitch_mm * 0.5;
            for (double x = xLo + xShift; x <= xHi + 1e-9; x += in.pitch_mm) {
                centers.emplace_back(x, y);
            }
        }
    }
    if (centers.empty())
        throw SkillError("fill_pattern: no instances generated");

    for (const auto& c : centers) {
        const gp_Pnt p(
            c.first  + ex.normal.X() * kOverhang,
            c.second + ex.normal.Y() * kOverhang,
            ex.centerZ + ex.normal.Z() * kOverhang);
        const gp_Ax2 ax(p, cutDir);
        cutters.push_back(pr::cylinder(ax, r, toolLen));
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    const int total = static_cast<int>(centers.size());
    const double perVol = M_PI * r * r * in.source_depth_mm;
    const double totalVol = perVol * total;

    json params = {
        { "face_id",            faceId },
        { "source_hole_dia_mm", in.source_hole_dia_mm },
        { "source_depth_mm",    in.source_depth_mm },
        { "fill_type",          in.fill_type },
        { "pitch_mm",           in.pitch_mm },
        { "margin_mm",          in.margin_mm },
    };
    json pattern = {
        { "kind",           kSkillId },
        { "is_pattern",     true },
        { "instance_count", total },
        { "derived_params", {
            { "fill_type",         in.fill_type },
            { "pitch_mm",          in.pitch_mm },
            { "source_hole_dia_mm", in.source_hole_dia_mm },
            { "source_depth_mm",   in.source_depth_mm },
            { "per_inst_vol_mm3",  perVol },
            { "total_vol_mm3",     totalVol },
        } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill";
    tooling.tool_dia_mm       = in.source_hole_dia_mm;
    tooling.tool_length_mm    = in.source_depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = totalVol;
    tooling.est_cycle_time_s  = std::max(1.0, in.source_depth_mm / 50.0) * total;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::fill_pattern applied: {} {}x with {} type",
                  total, in.pitch_mm, in.fill_type);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Heuristic: count cylindrical faces of equal radius (≥ 6) — flag as a fill.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 0.95;
        r.matched_geometry = f.pattern;
        out.push_back(r);
    }
    if (!out.empty()) return out;

    std::vector<double> radii;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder gc = s.Cylinder();
            radii.push_back(gc.Radius());
        } catch (...) {}
    }
    if (radii.size() < 6) return out;

    // Pick dominant radius (most common bucket).
    std::sort(radii.begin(), radii.end());
    int bestCount = 1, run = 1;
    double bestR = radii.front();
    for (size_t i = 1; i < radii.size(); ++i) {
        if (std::abs(radii[i] - radii[i - 1]) < 1e-2) {
            ++run;
            if (run > bestCount) { bestCount = run; bestR = radii[i]; }
        } else {
            run = 1;
        }
    }
    if (bestCount < 6) return out;

    json recovered = {
        { "face_id",            -1 },
        { "source_hole_dia_mm", 2.0 * bestR },
        { "source_depth_mm",    0.0 },
        { "fill_type",          "grid" },
        { "pitch_mm",           0.0 },
        { "margin_mm",          0.0 },
    };
    json matched = { { "cyl_count", bestCount }, { "dom_radius", bestR } };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    return out;
}

}  // namespace koocadcam::skill::fill_pattern
