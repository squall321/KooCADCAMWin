// @lat: [[engine/cam-3axis-verify#Stage 4]]
//
// Slice-1 collision check.  See CollisionCheck.hpp for the rationale.

#include "CollisionCheck.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace koocadcam::cam {

namespace {

// Linear interpolation along a segment.  Arcs are treated as a straight
// line from prev → end for the purposes of sampling (slice-1 simplification).
gp_Pnt lerp(const gp_Pnt& a, const gp_Pnt& b, double t)
{
    return gp_Pnt(a.X() + (b.X() - a.X()) * t,
                  a.Y() + (b.Y() - a.Y()) * t,
                  a.Z() + (b.Z() - a.Z()) * t);
}

// Squared XY distance.
double xyDist(const gp_Pnt& p, double cx, double cy)
{
    const double dx = p.X() - cx;
    const double dy = p.Y() - cy;
    return std::sqrt(dx * dx + dy * dy);
}

// Test whether the tool (radius = toolR, length = toolLen) at tip position
// `tip` collides with the stock bbox + margin.
//
// Heuristic:
//   - The "shank column" runs from tip.Z to tip.Z + toolLen.
//   - The shank XY footprint is a disc of radius toolR around (tip.X, tip.Y).
//   - The stock bbox is [xMin..xMax] × [yMin..yMax] × [zMin..zMax + margin].
//
// Collision if BOTH:
//   (a) the shank column overlaps the stock Z extent above the tip's cut
//       depth (tip.Z < zMax + margin AND tip.Z + toolLen > zMin) AND
//   (b) the tool disc XY centre is inside the stock XY footprint expanded
//       by toolR (i.e., the disc actually overlaps the bbox in XY).
//
// (b) uses an axis-aligned-rectangle test, conservative for a circular disc
// vs. a rectangular bbox but adequate for slice-1.
bool collidesAt(const gp_Pnt& tip,
                double        toolR,
                double        toolLen,
                double        xMin, double yMin, double zMin,
                double        xMax, double yMax, double zMax,
                double        safe_z_margin)
{
    // (a) Z overlap test.
    if (tip.Z() >= zMax + safe_z_margin) return false;          // tip above stock top — clear
    if (tip.Z() + toolLen <= zMin)        return false;          // entire tool below stock — silly, but safe
    // (b) XY footprint test (rectangle expanded by toolR).
    const double xLo = xMin - toolR;
    const double xHi = xMax + toolR;
    const double yLo = yMin - toolR;
    const double yHi = yMax + toolR;
    if (tip.X() < xLo || tip.X() > xHi) return false;
    if (tip.Y() < yLo || tip.Y() > yHi) return false;
    // Refinement: if the tool tip is at or below zMax+margin AND inside
    // the XY footprint, that's a collision.  We also catch the case of
    // a Rapid that overflies stock at safe_z without diving — the Z gate
    // above handles it.
    return true;
}

// Number of sample steps for a segment.
int stepsFor(double length_mm, double step_mm)
{
    if (length_mm <= 0.0 || step_mm <= 0.0) return 1;
    return std::max(1, static_cast<int>(std::ceil(length_mm / step_mm)));
}

}  // namespace

std::vector<CollisionEvent> checkPath(const Toolpath&         path,
                                       const skill::Workpiece& wp,
                                       double                  sample_step_mm,
                                       double                  safe_z_margin)
{
    std::vector<CollisionEvent> out;
    if (path.segments.empty()) return out;

    // Compute workpiece bbox once.  We assume wp is a non-null shape; if the
    // shape is null we just return empty.
    if (wp.shape().IsNull()) {
        spdlog::warn("cam::checkPath: workpiece shape is null; skipping check");
        return out;
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double toolR   = path.tool_dia_mm    * 0.5;
    const double toolLen = path.tool_length_mm > 0.0 ? path.tool_length_mm : 20.0;

    // Iterate segments.  We need the "previous" end-point as the start of
    // each segment; segment 0 has no defined start, so we treat its
    // end_point as a single sample.
    gp_Pnt prevEnd = path.segments.front().end_point;
    bool first = true;

    for (size_t i = 0; i < path.segments.size(); ++i) {
        const auto& seg = path.segments[i];
        if (first) {
            // Sample only the end-point of segment 0.
            if (collidesAt(seg.end_point, toolR, toolLen,
                           xMin, yMin, zMin, xMax, yMax, zMax, safe_z_margin)) {
                std::ostringstream desc;
                desc << "segment " << i << " (initial) tip at "
                     << seg.end_point.X() << "," << seg.end_point.Y()
                     << "," << seg.end_point.Z()
                     << " is inside stock bbox";
                out.push_back({static_cast<int>(i), 0.0, desc.str()});
            }
            prevEnd = seg.end_point;
            first = false;
            continue;
        }

        const double segLen = prevEnd.Distance(seg.end_point);
        const int    nSteps = stepsFor(segLen, sample_step_mm);

        for (int k = 1; k <= nSteps; ++k) {
            const double t = static_cast<double>(k) / static_cast<double>(nSteps);
            const gp_Pnt sample = lerp(prevEnd, seg.end_point, t);
            if (collidesAt(sample, toolR, toolLen,
                           xMin, yMin, zMin, xMax, yMax, zMax, safe_z_margin)) {
                std::ostringstream desc;
                desc << "segment " << i
                     << " (" << (seg.move == PathSegment::Move::Rapid ? "Rapid" : "Cut")
                     << ") sample t=" << t
                     << " tip Z=" << sample.Z()
                     << " inside stock bbox (zMax=" << zMax << ")";
                out.push_back({static_cast<int>(i), t, desc.str()});
                break;  // one event per segment is enough for slice-1
            }
        }

        prevEnd = seg.end_point;
    }

    return out;
}

}  // namespace koocadcam::cam
