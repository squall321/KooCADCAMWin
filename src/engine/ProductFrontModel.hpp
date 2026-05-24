#pragma once
// @lat: [[engine/parametric-templates#ProductFrontModel]]

#include <TopoDS_Shape.hxx>

#include <string>
#include <vector>

namespace koocadcam::engine {

struct BuildWarning
{
    std::string code;
    std::string message;
};

struct StepResult
{
    TopoDS_Shape shape;
    std::vector<BuildWarning> warnings;
};

// Marker base for parametric product front models (CRTP).
// Concrete derived classes (WatchFrontModel, PhoneFrontModel) define their
// own step methods; the base intentionally has no virtual or templated step
// wrappers — the step sequence varies per product domain.
template <typename Derived>
class ProductFrontModel
{
protected:
    ProductFrontModel() = default;
    ~ProductFrontModel() = default;
};

}  // namespace koocadcam::engine
