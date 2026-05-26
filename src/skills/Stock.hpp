#pragma once
// @lat: [[engine/skills#Stock]]
//
// Workpiece stock factories.  A stock is the starting workpiece (raw
// material) before any skill is applied.
//
// For phase-1, the supported stocks are simple analytic primitives:
//   - cuboid (rectangular aluminum block)
//   - cylindrical (round bar / puck)
//
// Future phases:
//   - extruded profile (custom 2D cross-section)
//   - imported STEP (existing part as starting workpiece)

#include "Workpiece.hpp"

#include <memory>
#include <string>

namespace koocadcam::skill {

// Cuboid stock — axis-aligned box from origin to (w, h, d).
// The block is positioned with its origin at (0, 0, 0) and extends along
// +X, +Y, +Z.  Top face is at Z = depth_mm.
std::shared_ptr<Workpiece> createCuboidStock(
    double width_mm,
    double height_mm,
    double depth_mm,
    const std::string& material = "aluminum_6061");

// Cylindrical stock — circular puck centered on Z axis.
std::shared_ptr<Workpiece> createCylindricalStock(
    double diameter_mm,
    double thickness_mm,
    const std::string& material = "aluminum_6061");

}  // namespace koocadcam::skill
