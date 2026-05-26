#pragma once
// @lat: [[engine/parametric-templates#primitives — StepGuard]]
//
// Uniform exception + BRepCheck wrapper for build steps.  Lets each step
// body focus on the geometry it builds; the wrapper handles the universal
// E_OCCT / W_BREPCHECK warning conventions and the post-step debug log.

#include "Bbox.hpp"
#include "../ProductFrontModel.hpp"

#include <BRepCheck_Analyzer.hxx>
#include <TopoDS_Shape.hxx>

#include <spdlog/spdlog.h>

#include <exception>
#include <string>
#include <utility>
#include <vector>

namespace koocadcam::engine::prim {

// Run a step body and wrap the result.
//   - The body is a callable `TopoDS_Shape(std::vector<BuildWarning>&)`.
//     It receives a warnings sink for non-fatal diagnostics.
//   - On exception → BuildWarning{"E_OCCT", what()} + null shape.
//   - On null shape return → propagated as null result.
//   - On `checkValidity=true` and invalid BRep → BuildWarning{"W_BREPCHECK"}
//     + null shape (so the step counts as failed).
//   - On success → spdlog::debug with optimal-bbox extents.
template <typename F>
inline StepResult runStep(const char* stepName, F&& body, bool checkValidity = true)
{
    std::vector<BuildWarning> warnings;
    TopoDS_Shape shape;
    try {
        shape = body(warnings);
    }
    catch (const std::exception& e) {
        spdlog::error("{} exception: {}", stepName, e.what());
        warnings.push_back(BuildWarning{ "E_OCCT", e.what() });
        return StepResult{ TopoDS_Shape(), std::move(warnings) };
    }
    if (shape.IsNull()) {
        spdlog::error("{} produced null shape", stepName);
        return StepResult{ TopoDS_Shape(), std::move(warnings) };
    }
    if (checkValidity) {
        BRepCheck_Analyzer checker(shape);
        if (!checker.IsValid()) {
            spdlog::warn("{}: BRepCheck invalid", stepName);
            warnings.push_back(BuildWarning{
                "W_BREPCHECK",
                std::string(stepName) + " produced invalid BRep"
            });
            return StepResult{ TopoDS_Shape(), std::move(warnings) };
        }
    }
    const Bbox3d b = optimalBbox(shape);
    spdlog::debug("{} ok  bbox dx={:.3f} dy={:.3f} dz={:.3f}",
                  stepName, b.dx(), b.dy(), b.dz());
    return StepResult{ shape, std::move(warnings) };
}

}  // namespace koocadcam::engine::prim
