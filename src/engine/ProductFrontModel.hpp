#pragma once
// @lat: [[engine/parametric-templates#ProductFrontModel]]

#include <TopoDS_Shape.hxx>

#include <optional>
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

// CRTP base for parametric product front models.
// Concrete builders: WatchFrontModel, PhoneFrontModel (future).
// M0 is a stub — concrete builders arrive in M1.1+.
template <typename Derived>
class ProductFrontModel
{
public:
    // Each builder step takes the current shape and returns a new shape.
    // Steps must be deterministic.

    [[nodiscard]] StepResult buildBase() { return derived().buildBaseImpl(); }
    [[nodiscard]] StepResult applyCornerRadius(double r) { return derived().applyCornerRadiusImpl(r); }

    // ... future: addFeatures(), runDFM(), exportSTEP()

protected:
    ProductFrontModel() = default;
    ~ProductFrontModel() = default;

private:
    Derived& derived() { return static_cast<Derived&>(*this); }
    const Derived& derived() const { return static_cast<const Derived&>(*this); }
};

}  // namespace koocadcam::engine
