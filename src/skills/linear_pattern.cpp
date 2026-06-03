// @lat: [[engine/skills#linear_pattern]]

#include "linear_pattern.hpp"

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

namespace koocadcam::skill::linear_pattern {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.count_x < 1) {
        r.add("DFM-INPUT", "error", "linear_pattern: count_x must be >= 1");
    }
    if (in.count_y < 1) {
        r.add("DFM-INPUT", "error", "linear_pattern: count_y must be >= 1");
    }
    if (in.hole_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "linear_pattern: hole_dia_mm must be > 0");
    }
    if (in.hole_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "linear_pattern: hole_depth_mm must be > 0");
    }
    // DFM-002 — minimum drill diameter (ISO 235:2016 HSS twist-drill floor).
    if (in.hole_dia_mm > 0.0 && in.hole_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "linear_pattern: hole_dia_mm " + std::to_string(in.hole_dia_mm) +
              " < 0.8 mm (ISO 235:2016 standard HSS drill floor)");
    }
    // DFM-PATT-1 — pitch must clear the hole diameter to avoid overlap.
    if (in.count_x > 1 && in.pitch_x_mm <= in.hole_dia_mm) {
        r.add("DFM-PATT-1", "error",
              "linear_pattern: pitch_x_mm " + std::to_string(in.pitch_x_mm) +
              " <= hole_dia_mm " + std::to_string(in.hole_dia_mm) +
              " (instances would overlap along X)");
    }
    if (in.count_y > 1 && in.pitch_y_mm <= in.hole_dia_mm) {
        r.add("DFM-PATT-1", "error",
              "linear_pattern: pitch_y_mm " + std::to_string(in.pitch_y_mm) +
              " <= hole_dia_mm " + std::to_string(in.hole_dia_mm) +
              " (instances would overlap along Y)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "linear_pattern DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceIdOpt = wp.resolve(in.face);
    if (!faceIdOpt) throw SkillError("linear_pattern: face datum unresolved");
    const int faceId = *faceIdOpt;
    const gp_Pnt center = wp.faceCenter(faceId);
    const gp_Dir normal = wp.faceNormal(faceId);

    // Tool length must overshoot the depth so the cut is clean at the entry.
    constexpr double kOverhang = 0.1;
    const double toolLen = in.hole_depth_mm + 2.0 * kOverhang;
    const double r = in.hole_dia_mm * 0.5;

    // Cut axis direction = INTO the face (opposite outward normal).
    const gp_Dir cutDir(-normal.X(), -normal.Y(), -normal.Z());

    const int total = in.count_x * in.count_y;
    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(static_cast<size_t>(total));

    // The face center is on the entry plane.  Start each cylinder slightly
    // outside the face along +normal, then sweep INTO the body for toolLen.
    for (int j = 0; j < in.count_y; ++j) {
        for (int i = 0; i < in.count_x; ++i) {
            const double dx = in.start_x_mm + i * in.pitch_x_mm;
            const double dy = in.start_y_mm + j * in.pitch_y_mm;
            const gp_Pnt entryOnPlane(
                center.X() + dx,
                center.Y() + dy,
                center.Z() + 0.0);
            // Lift slightly along +normal to start the cylinder above the face.
            const gp_Pnt cylStart(
                entryOnPlane.X() + normal.X() * kOverhang,
                entryOnPlane.Y() + normal.Y() * kOverhang,
                entryOnPlane.Z() + normal.Z() * kOverhang);
            const gp_Ax2 ax(cylStart, cutDir);
            cutters.push_back(pr::cylinder(ax, r, toolLen));
        }
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    const double perVol = M_PI * r * r * in.hole_depth_mm;
    const double totalVol = perVol * total;

    json params = {
        { "face_id",        faceId },
        { "hole_dia_mm",    in.hole_dia_mm },
        { "hole_depth_mm",  in.hole_depth_mm },
        { "count_x",        in.count_x },
        { "pitch_x_mm",     in.pitch_x_mm },
        { "count_y",        in.count_y },
        { "pitch_y_mm",     in.pitch_y_mm },
        { "start_x_mm",     in.start_x_mm },
        { "start_y_mm",     in.start_y_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_pattern",           true },
        { "instance_count",       total },
        { "derived_params", {
            { "hole_dia_mm",      in.hole_dia_mm },
            { "hole_depth_mm",    in.hole_depth_mm },
            { "pitch_x_mm",       in.pitch_x_mm },
            { "pitch_y_mm",       in.pitch_y_mm },
            { "count_x",          in.count_x },
            { "count_y",          in.count_y },
            { "per_inst_vol_mm3", perVol },
            { "total_vol_mm3",    totalVol },
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
    wpNew->addFeature(sig);

    spdlog::debug("skill::linear_pattern applied: {}x{} dia={} depth={} vol={}",
                  in.count_x, in.count_y, in.hole_dia_mm,
                  in.hole_depth_mm, totalVol);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Heuristic: group all cylindrical faces by (axis-direction, radius).  If
// any group of ≥2 has cylinders whose XY centers lie on a colinear set with
// a constant pitch, mark it a linear pattern candidate.

namespace {

struct CylInst
{
    double x, y, z;
    double radius;
    gp_Dir dir;
};

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay.
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

    std::vector<CylInst> cyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            CylInst ci{};
            ci.x = c.Location().X();
            ci.y = c.Location().Y();
            ci.z = c.Location().Z();
            ci.radius = c.Radius();
            ci.dir = c.Axis().Direction();
            cyls.push_back(ci);
        } catch (...) { continue; }
    }

    if (cyls.size() < 2) return out;

    // Pair-finder: find ≥2 cyls with same radius (±1e-2) whose XY positions
    // share a non-trivial Δx or Δy.
    for (size_t a = 0; a < cyls.size(); ++a) {
        for (size_t b = a + 1; b < cyls.size(); ++b) {
            if (std::abs(cyls[a].radius - cyls[b].radius) > 1e-2) continue;
            const double dx = cyls[b].x - cyls[a].x;
            const double dy = cyls[b].y - cyls[a].y;
            const double dist = std::sqrt(dx * dx + dy * dy);
            if (dist < cyls[a].radius * 2.0) continue;  // not separable

            json recovered = {
                { "hole_dia_mm",   2.0 * cyls[a].radius },
                { "hole_depth_mm", 0.0 },
                { "count_x",       static_cast<int>(cyls.size()) },
                { "pitch_x_mm",    std::abs(dx) },
                { "count_y",       1 },
                { "pitch_y_mm",    0.0 },
                { "start_x_mm",    cyls[a].x },
                { "start_y_mm",    cyls[a].y },
            };
            json matched = {
                { "cyl_count",    static_cast<int>(cyls.size()) },
                { "first_radius", cyls[a].radius },
            };
            out.push_back(RecognizedFeature{
                kSkillId, recovered, /*confidence*/ 0.55, matched });
            return out;
        }
    }
    return out;
}

}  // namespace koocadcam::skill::linear_pattern
