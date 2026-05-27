#pragma once
// @lat: [[engine/skills#Layer 6 Parts layout]]
//
// Part — one internal part in a KooCADCAM layout.
//
// A `Part` is an entity that lives INSIDE the eventual machined frame
// (e.g. PCB, battery, display, crown shaft).  Skills do not cut "parts" —
// instead the position/AABB of each Part is used as a SOURCE OF TRUTH for
// datum-driven skill positioning.  When a Part moves or its AABB resizes,
// the dependency graph (see DatumGraph.hpp) maps it to the (re)affected
// process plan steps so they can be re-evaluated.
//
// Fields
// ──────
//   id         : stable identifier (filename stem or user-supplied).  Used as
//                the lookup key in PartsLayout::findById and as the edge
//                label in DatumGraph.
//   step_path  : path to the STEP file that produced `shape` (or empty if
//                the Part was injected programmatically — useful for tests).
//   shape      : the actual OCCT shape (may be a compound when the STEP file
//                contained multiple roots).
//   xMin..zMax : cached axis-aligned bounding box of `shape` AFTER applying
//                `placement`.  Computed once at construction so layout
//                queries (centre, intersection, nearest-to-point) avoid
//                paying for a fresh BRepBndLib::AddOptimal on every call.
//   placement  : optional rigid-body transformation; defaults to identity.
//                When non-trivial, the AABB stored above reflects the
//                placed shape, not the raw `shape` field.
//
// `Part` is a value type — copy / move freely.  The OCCT `TopoDS_Shape` is
// already reference-counted internally so copying is cheap.

#include <TopoDS_Shape.hxx>
#include <gp_Trsf.hxx>

#include <string>

namespace koocadcam::parts {

struct Part
{
    std::string   id;             // "crown_shaft", "display", "pcb_main", ...
    std::string   step_path;      // path to STEP file (may be empty)
    TopoDS_Shape  shape;          // OCCT shape (compound or solid)

    // Cached AABB of (placement * shape).  Computed once in PartsLayout
    // helpers; set to all-zero on a default-constructed Part.
    double xMin = 0.0, yMin = 0.0, zMin = 0.0;
    double xMax = 0.0, yMax = 0.0, zMax = 0.0;

    // Optional placement transformation (default identity).
    gp_Trsf placement;

    // ── Convenience accessors ────────────────────────────────────────────

    // Centre of the AABB.  Cheap — no OCCT calls.
    double centerX() const { return 0.5 * (xMin + xMax); }
    double centerY() const { return 0.5 * (yMin + yMax); }
    double centerZ() const { return 0.5 * (zMin + zMax); }

    double sizeX() const { return xMax - xMin; }
    double sizeY() const { return yMax - yMin; }
    double sizeZ() const { return zMax - zMin; }
};

}  // namespace koocadcam::parts
