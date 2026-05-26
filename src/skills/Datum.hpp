#pragma once
// @lat: [[engine/skills#Datum system]]
//
// Hybrid datum references.  A skill input can specify a face / edge / vertex
// either:
//   (a) Explicitly — by topological index in the workpiece's TopExp_Explorer
//       enumeration.  Used when the caller knows exactly which face.
//   (b) Inferred  — by geometric criterion (normal direction, largest area,
//       ray hit, …).  Resolved against the current workpiece at apply-time.
//
// Both forms coexist in the same std::variant so callers can mix freely:
//
//   FaceDatum entry = FaceByNormal{ gp::DZ() };       // inferred top face
//   FaceDatum entry = FaceIdRef{ 3 };                  // explicit
//
// Resolution is performed by `Workpiece::resolve(datum)`.

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <string>
#include <variant>

namespace koocadcam::skill {

// ── Explicit references (by enumeration index) ───────────────────────────
struct FaceIdRef   { int face_id;   };
struct EdgeIdRef   { int edge_id;   };
struct VertexIdRef { int vertex_id; };

// ── Inferred face references ─────────────────────────────────────────────

// "The face whose outward normal is closest to `target_normal`, within
// `tolerance_deg`.  Variant chooses among multiple matches: any | largest |
// smallest."
struct FaceByNormal
{
    gp_Dir       target_normal;
    double       tolerance_deg = 5.0;
    std::string  variant       = "largest";   // "any" | "largest" | "smallest"
};

// "The largest planar face on the workpiece" (Z-agnostic).
struct FaceLargestPlanar {};

// "The face hit first by a ray from `origin` in `direction`."
struct FaceByRay
{
    gp_Pnt origin;
    gp_Dir direction;
};

// "The cylindrical face whose axis is closest to `axis` (within tolerance)."
struct FaceCylinderByAxis
{
    gp_Ax1 axis;
    double tolerance_deg = 5.0;
};

// "The face at the boundary between the workpiece and 'air' in the +Z
//  direction, projected from `xy_point`."
//  i.e. top of a stepped surface at a given XY.
struct FaceTopAtXY
{
    double x_mm;
    double y_mm;
};

// ── Inferred edge references ─────────────────────────────────────────────

struct EdgeAtZ
{
    double z_value;
    double tolerance_mm = 1e-3;
};

struct EdgeByCircle
{
    double radius_mm;
    double tolerance_mm = 1e-3;
};

struct EdgeBetweenFaces
{
    FaceIdRef face_a;
    FaceIdRef face_b;
};

// ── Inferred vertex references ───────────────────────────────────────────

struct VertexAtPoint
{
    gp_Pnt    target;
    double    tolerance_mm = 1e-3;
};

// ── Variant unions ────────────────────────────────────────────────────────

using FaceDatum = std::variant<
    FaceIdRef,
    FaceByNormal,
    FaceLargestPlanar,
    FaceByRay,
    FaceCylinderByAxis,
    FaceTopAtXY>;

using EdgeDatum = std::variant<
    EdgeIdRef,
    EdgeAtZ,
    EdgeByCircle,
    EdgeBetweenFaces>;

using VertexDatum = std::variant<
    VertexIdRef,
    VertexAtPoint>;

}  // namespace koocadcam::skill
