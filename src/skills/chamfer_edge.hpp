#pragma once
// @lat: [[engine/skills#chamfer_edge]]
//
// chamfer_edge — bevel one or more edges with a flat angled face.
//
//   Before:                       After (chamfer_size = c, angle = 45°):
//
//        ┌─────────┐                ┌──────────┐
//        │         │                │\         │
//        │         │                │ \  ← flat planar face
//        │         │                │  \       │
//        │         │                │   \      │
//        └─────────┘                └────\─────┘
//
// As with fillet_edge, the edge selector is hybrid — either a single
// EdgeDatum, or a Z-band (`edges_at_z_mm` + `tolerance_mm`).
//
// `angle_deg` controls how the bevel is split between the two adjacent
// faces.  45° gives a symmetric chamfer; for asymmetric chamfers (e.g. an
// edge break that protects the top face more than the side), values from
// 30° to 60° are typical.  We implement angle via OCCT's symmetric
// `BRepFilletAPI_MakeChamfer::Add(distance, edge)` parameterisation —
// when `angle_deg != 45`, we adjust the equivalent distance per the
// trigonometric relation between the two faces.  In our implementation,
// for /WX-safe simplicity, we use the symmetric Add() (equal distance on
// both faces) and treat `angle_deg` as a future-extension parameter that
// is recorded in the signature but not yet honoured beyond a [15,75]
// range check.  This matches drill_hole's pragmatic phase-1 stance.
//
// Signature pattern:
//   - 1 planar face per chamfered edge (the new bevel face).
//   - The planar normal lies between the normals of the two former
//     adjacent faces, roughly bisecting them at `angle_deg`.
//
// DFM checks:
//   chamfer_size_mm ≥ 0.1 mm  (min realistic edge break)
//   angle_deg ∈ [15°, 75°]   (extreme angles approach knife-edge → DFM-011)
//
// Recognize:
//   Locate planar faces whose normal is between the normals of two other
//   planar faces sharing an edge.  Confidence based on how close the
//   half-angle is to ~ 45° and how small the face area is.

#include "Datum.hpp"
#include "Skill.hpp"
#include "fillet_edge.hpp"   // for EdgesAtZBand / EdgeSelector type

#include <memory>
#include <variant>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace chamfer_edge {

constexpr const char* kSkillId = "chamfer_edge";

// We re-use the same hybrid edge selector definition as fillet_edge so the
// caller can mix Z-band and explicit datum selectors uniformly.
using EdgeSelector = fillet_edge::EdgeSelector;
using EdgesAtZBand = fillet_edge::EdgesAtZBand;

struct Input
{
    EdgeSelector edge_selector;
    double       chamfer_size_mm = 0.0;   // distance along each adjacent face
    double       angle_deg       = 45.0;  // bevel angle; range 15-75 (default 45)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace chamfer_edge
}  // namespace koocadcam::skill
