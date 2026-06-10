// @lat: [[engine/skills#sketch_driven_pattern]]

#include "sketch_driven_pattern.hpp"

#include "Workpiece.hpp"
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

namespace koocadcam::skill::sketch_driven_pattern {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.positions.empty()) {
        r.add("DFM-INPUT", "error",
              "sketch_driven_pattern: positions vector is empty");
    }
    if (in.hole_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "sketch_driven_pattern: hole_dia_mm must be > 0");
    }
    if (in.hole_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "sketch_driven_pattern: hole_depth_mm must be > 0");
    }
    if (in.hole_dia_mm > 0.0 && in.hole_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "sketch_driven_pattern: hole_dia_mm " +
              std::to_string(in.hole_dia_mm) +
              " < 0.8 mm (ISO 235:2016)");
    }
    // DFM-PATT-5 — every pair must clear hole_dia.
    if (in.hole_dia_mm > 0.0) {
        for (size_t i = 0; i < in.positions.size(); ++i) {
            for (size_t j = i + 1; j < in.positions.size(); ++j) {
                const double dx = in.positions[j].first  - in.positions[i].first;
                const double dy = in.positions[j].second - in.positions[i].second;
                const double d  = std::sqrt(dx * dx + dy * dy);
                if (d <= in.hole_dia_mm) {
                    r.add("DFM-PATT-5", "error",
                          "sketch_driven_pattern: positions[" +
                          std::to_string(i) + "] and [" + std::to_string(j) +
                          "] separated by " + std::to_string(d) +
                          " <= hole_dia_mm " +
                          std::to_string(in.hole_dia_mm) +
                          " (overlap)");
                    return r;   // one is enough
                }
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
        std::string msg = "sketch_driven_pattern DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceIdOpt = wp.resolve(in.entry_face);
    if (!faceIdOpt) throw SkillError("sketch_driven_pattern: face unresolved");
    const int faceId = *faceIdOpt;
    const gp_Pnt center = wp.faceCenter(faceId);
    const gp_Dir normal = wp.faceNormal(faceId);
    const gp_Dir cutDir(-normal.X(), -normal.Y(), -normal.Z());

    constexpr double kOverhang = 0.1;
    const double toolLen = in.hole_depth_mm + 2.0 * kOverhang;
    const double r = in.hole_dia_mm * 0.5;

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(in.positions.size());
    for (const auto& xy : in.positions) {
        const gp_Pnt p(
            center.X() + xy.first  + normal.X() * kOverhang,
            center.Y() + xy.second + normal.Y() * kOverhang,
            center.Z() +             normal.Z() * kOverhang);
        const gp_Ax2 ax(p, cutDir);
        cutters.push_back(pr::cylinder(ax, r, toolLen));
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    const int total = static_cast<int>(in.positions.size());
    const double perVol = M_PI * r * r * in.hole_depth_mm;
    const double totalVol = perVol * total;

    json posArr = json::array();
    for (const auto& xy : in.positions)
        posArr.push_back({ { "x", xy.first }, { "y", xy.second } });

    json params = {
        { "hole_dia_mm",    in.hole_dia_mm },
        { "hole_depth_mm",  in.hole_depth_mm },
        { "positions",      posArr },
        { "entry_face_id",  faceId },
    };
    json pattern = {
        { "kind",           kSkillId },
        { "is_pattern",     true },
        { "instance_count", total },
        { "derived_params", {
            { "hole_dia_mm",       in.hole_dia_mm },
            { "hole_depth_mm",     in.hole_depth_mm },
            { "position_count",    total },
            { "per_inst_vol_mm3",  perVol },
            { "total_vol_mm3",     totalVol },
        } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill";
    tooling.tool_dia_mm       = in.hole_dia_mm;
    tooling.tool_length_mm    = in.hole_depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = totalVol;
    tooling.est_cycle_time_s  = std::max(1.0, in.hole_depth_mm / 50.0) * total;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::sketch_driven_pattern applied: N={} dia={} vol={}",
                  total, in.hole_dia_mm, totalVol);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Heuristic: ≥3 cylindrical faces with identical radius but no obvious
// linear/circular pattern → flag as sketch-driven.  Here we surface a
// candidate whenever ≥3 same-radius cylinders exist, with low confidence.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 0.92;
        r.matched_geometry = f.pattern;
        out.push_back(r);
    }
    if (!out.empty()) return out;

    struct C { double x, y, r; };
    std::vector<C> cs;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder gc = s.Cylinder();
            cs.push_back(C{ gc.Location().X(), gc.Location().Y(), gc.Radius() });
        } catch (...) {}
    }
    if (cs.size() < 3) return out;

    // Pick dominant radius bucket.
    std::sort(cs.begin(), cs.end(),
              [](const C& a, const C& b) { return a.r < b.r; });
    int bestCount = 1, run = 1;
    double bestR = cs.front().r;
    size_t bestStart = 0, runStart = 0;
    for (size_t i = 1; i < cs.size(); ++i) {
        if (std::abs(cs[i].r - cs[i - 1].r) < 1e-2) {
            ++run;
            if (run > bestCount) {
                bestCount = run;
                bestR = cs[i].r;
                bestStart = runStart;
            }
        } else {
            run = 1;
            runStart = i;
        }
    }
    if (bestCount < 3) return out;

    json posArr = json::array();
    for (int i = 0; i < bestCount; ++i) {
        posArr.push_back({
            { "x", cs[bestStart + i].x },
            { "y", cs[bestStart + i].y },
        });
    }
    json recovered = {
        { "hole_dia_mm",   2.0 * bestR },
        { "hole_depth_mm", 0.0 },
        { "positions",     posArr },
        { "entry_face_id", -1 },
    };
    json matched = { { "cyl_count", bestCount }, { "dom_radius", bestR } };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.5, matched });
    return out;
}

}  // namespace koocadcam::skill::sketch_driven_pattern
