#pragma once
// @lat: [[engine/feature-watch#Build Sequence]]
// M0 stand-in for the future WatchFrontModel: a simple cylinder
// of watch-baseline size (ø30 mm × 5 mm thick).

#include <TopoDS_Shape.hxx>

namespace koocadcam::engine {

class PlaceholderCylinder
{
public:
    static constexpr double kDefaultDiameterMm = 30.0;
    static constexpr double kDefaultHeightMm   = 5.0;

    // Build a cylinder of the given diameter and height (both mm).
    // Throws Standard_Failure on OCCT API errors.
    [[nodiscard]] static TopoDS_Shape make(
        double diameterMm = kDefaultDiameterMm,
        double heightMm   = kDefaultHeightMm);
};

}  // namespace koocadcam::engine
