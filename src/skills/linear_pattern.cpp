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
#include <numeric>
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
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
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

    // ── Geometric path B (foreign geometry) ──────────────────────────────
    // A linear pattern is a regular GRID of identical, coaxial cylinders.  The
    // old fallback fired on ANY two same-radius cylinders and flattened them to
    // a 1-D count — so an unrelated pair of holes was mis-read as a pattern and
    // a 2-D grid was mangled.  We instead REQUIRE a real equally-spaced grid:
    // the largest set of same-radius / same-axis cylinders whose centres fall on
    // a 1-D (or 2-D) lattice with a consistent pitch.  At least 3 instances and
    // < 5 % pitch variation are needed, so two coincidental holes never qualify.
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
    if (cyls.size() < 3) return out;

    // Group by (radius, axis direction): only co-radial, co-axial cylinders can
    // belong to one pattern.
    auto sameGroup = [](const CylInst& a, const CylInst& b) {
        return std::abs(a.radius - b.radius) < 1e-2 &&
               std::abs(std::abs(a.dir.Dot(b.dir)) - 1.0) < 1e-2;
    };
    std::vector<bool> used(cyls.size(), false);
    size_t bestCount = 0;
    json bestRecovered, bestMatched;

    for (size_t g = 0; g < cyls.size(); ++g) {
        if (used[g]) continue;
        std::vector<CylInst> grp;
        for (size_t k = 0; k < cyls.size(); ++k)
            if (!used[k] && sameGroup(cyls[g], cyls[k])) grp.push_back(cyls[k]);
        for (size_t k = 0; k < cyls.size(); ++k)
            if (!used[k] && sameGroup(cyls[g], cyls[k])) used[k] = true;
        if (grp.size() < 3) continue;

        // AXIS gate: the recovered params (XY pitch grid) and apply()'s
        // face-normal model assume holes bored along +/-Z.  A SIDE grille
        // (axis along X/Y — e.g. a watch speaker grille) would be recovered with
        // its Z stripped (hole_centers carries only XY), so its replay/subsumption
        // would be wrong.  Recover ONLY vertical-axis grids; a side grid is left
        // to its individual drills (a fuller 3-D pattern model is a follow-up).
        if (std::abs(grp[0].dir.Z()) < 0.99) continue;

        // Project centres onto the dominant axis to test for an equally-spaced
        // 1-D line.  The dominant direction is the largest XY spread axis.
        double minX = grp[0].x, maxX = grp[0].x, minY = grp[0].y, maxY = grp[0].y;
        for (const auto& c : grp) {
            minX = std::min(minX, c.x); maxX = std::max(maxX, c.x);
            minY = std::min(minY, c.y); maxY = std::max(maxY, c.y);
        }
        const double spanX = maxX - minX, spanY = maxY - minY;
        const bool xDominant = spanX >= spanY;

        // Sort along the dominant axis and measure inter-hole gaps.
        std::vector<CylInst> line = grp;
        std::sort(line.begin(), line.end(),
            [xDominant](const CylInst& a, const CylInst& b) {
                return xDominant ? a.x < b.x : a.y < b.y;
            });
        // Collinearity: the off-axis spread must be small relative to the on-axis
        // span (otherwise it's a 2-D blob, not a clean line — punt to circular).
        const double onSpan  = xDominant ? spanX : spanY;
        const double offSpan = xDominant ? spanY : spanX;
        if (onSpan < grp[0].radius) continue;            // degenerate
        if (offSpan > 0.10 * onSpan) continue;           // not collinear

        // Equal pitch check.
        std::vector<double> gaps;
        for (size_t k = 1; k < line.size(); ++k)
            gaps.push_back(xDominant ? (line[k].x - line[k-1].x)
                                     : (line[k].y - line[k-1].y));
        const double meanGap =
            std::accumulate(gaps.begin(), gaps.end(), 0.0) / gaps.size();
        if (meanGap < 2.0 * grp[0].radius) continue;     // overlapping / bogus
        double maxDev = 0.0;
        for (double gp : gaps) maxDev = std::max(maxDev, std::abs(gp - meanGap));
        if (maxDev / meanGap > 0.05) continue;           // not equally spaced

        if (line.size() <= bestCount) continue;
        bestCount = line.size();

        json centers = json::array();
        for (const auto& c : line) centers.push_back({ c.x, c.y });
        const double pitch = xDominant ? meanGap : 0.0;
        const double pitchY = xDominant ? 0.0 : meanGap;
        bestRecovered = {
            { "hole_dia_mm",   2.0 * line[0].radius },
            { "hole_depth_mm", 0.0 },
            { "count_x",       xDominant ? static_cast<int>(line.size()) : 1 },
            { "pitch_x_mm",    pitch },
            { "count_y",       xDominant ? 1 : static_cast<int>(line.size()) },
            { "pitch_y_mm",    pitchY },
            { "start_x_mm",    line.front().x },
            { "start_y_mm",    line.front().y },
            { "hole_centers",  centers },     // for pattern subsumption
        };
        bestMatched = {
            { "cyl_count",      static_cast<int>(line.size()) },
            { "first_radius",   line[0].radius },
            { "pitch_mm",       meanGap },
            { "pitch_dev_frac", meanGap > 0 ? maxDev / meanGap : 0.0 },
        };
    }

    // Emit only at >= 4 instances: with 3 holes the pitch-consistency gate has
    // just 2 gaps and is near-vacuous, so a chance trio of equally-spaced
    // same-radius holes would pass.  Four instances force 3 consistent gaps —
    // strong enough to keep full (precise-tier) confidence and then subsume.
    if (bestCount >= 4)
        out.push_back(RecognizedFeature{
            kSkillId, bestRecovered, /*confidence*/ 0.7, bestMatched });
    return out;
}

}  // namespace koocadcam::skill::linear_pattern
