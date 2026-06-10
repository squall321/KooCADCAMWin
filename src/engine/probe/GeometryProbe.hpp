#pragma once
// @lat: [[engine/dfm-rules#GeometryProbe]]
// Product-agnostic DFM geometry probes (breakthrough plan B5.2-B5.3).
//
// Pure measurement functions over TopoDS shapes — no spec / product-model
// dependency, so WatchFrontModel, PhoneFrontModel and the RE adapters can
// share the exact same geometric rulers.  Each probe returns a scalar in
// mm / degrees; "no measurement possible" is signalled by a sentinel
// (NaN for single-pair probes, +inf for shape-wide aggregates), never by
// an exception.

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

namespace koocadcam::engine::probe {

// Wedge (knife-edge) angle between two faces, in degrees: 180° − interior
// angle of the midpoint-sampled outward normals.  dot ≈ +1 → faces
// near-coplanar → wedge ≈ 180° (fine); dot ≈ -1 → faces back-to-back →
// wedge ≈ 0° (knife).  The midpoint-normal wedge is only EXACT for two
// PLANAR faces, so any pair involving a curved face returns NaN (curved
// dihedrals need edge-evaluated normals: Phase 2).  Degenerate faces also
// return NaN.
[[nodiscard]] double dihedralWedgeDeg(const TopoDS_Face& fA,
                                      const TopoDS_Face& fB);

// DFM-011 probe: walk shared edges; compute the wedge angle between each
// adjacent planar-planar face pair via dihedralWedgeDeg.  Returns the
// smallest wedge (in degrees) observed across the shape — small angles
// indicate a knife edge.  Tangent-continuous pairs (wedge > 175°) and
// curved pairs are skipped.  +inf when nothing measurable (single-face
// shape, all-curved shape).
[[nodiscard]] double minDihedralAngleDeg(const TopoDS_Shape& s);

// B5.3 probe: minimum clearance between a cylindrical hole wall and the
// rest of the shape's faces, excluding `holeFace` itself and every face
// that shares an edge with it (the pierced top/bottom faces, chamfer
// rings, pocket floors).  Drives hole-too-close-to-outer-wall DFM rules.
// Returns +inf when `holeFace` is not cylindrical or no candidate face
// remains.
[[nodiscard]] double holeToOuterMinDistance(const TopoDS_Shape& shape,
                                            const TopoDS_Face& holeFace);

}  // namespace koocadcam::engine::probe
