#pragma once
// @lat: [[engine/feature-watch#JSON Schema]]
//
// Product registry — the unified, product-tag-driven entry point.
//
// Watch and Phone were two independent builders with no shared dispatch: every
// caller hard-coded WatchFrontModel:: or PhoneFrontModel:: (and the GUI used a
// `phone ? ... : ...` bool branch).  A recovered/adapted design carries a spec
// but not a C++ product type, so it had no way to be built.  This registry
// closes that: a spec's "product" field (or inferred keys) selects the builder,
// so a GENERIC caller builds + validates ANY product from a tagged spec.
//
// This is the build-side companion to dfm::runDFMForSpec (the DFM-side bridge):
// together they let product identity flow through the pipeline.

#include "PhoneFrontModel.hpp"
#include "ProductFrontModel.hpp"
#include "WatchFrontModel.hpp"
#include "dfm/ProductDFM.hpp"

#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace koocadcam::engine {

// The products this build knows how to make.
inline std::vector<std::string> productList()
{
    return { "watch", "phone" };
}

// Resolve a spec's product: explicit "product" field, else inferred from
// discriminating keys (kept in sync with dfm::runDFMForSpec).  Defaults to
// "watch" so an untagged minimal spec still builds something.
inline std::string productFromSpec(const nlohmann::json& spec)
{
    const std::string explicitTag = spec.value("product", std::string{});
    if (!explicitTag.empty()) return explicitTag;
    if (spec.contains("cameras") || spec.contains("port_hole") ||
        spec.contains("camera_deco_rings"))
        return "phone";
    if (spec.contains("bezel") || spec.contains("crown_cavity") ||
        spec.contains("lugs"))
        return "watch";
    return "watch";
}

// The canonical default spec for a product tag.
inline nlohmann::json defaultSpecForProduct(const std::string& product)
{
    if (product == "phone") return PhoneFrontModel::defaultSpec();
    return WatchFrontModel::defaultSpec();
}

// Build ANY product from a tagged/inferred spec — the entry point a generic
// caller (RE/adapt, GUI) uses instead of a hard-coded builder.
inline TopoDS_Shape buildProduct(const nlohmann::json& spec,
                                 std::vector<BuildWarning>& warnings)
{
    if (productFromSpec(spec) == "phone")
        return PhoneFrontModel::buildAll(spec, warnings);
    return WatchFrontModel::buildAll(spec, warnings);
}

// Product-aware DFM (delegates to the already-product-aware DFM bridge).
inline DFMReport runDFMForProduct(const TopoDS_Shape& shape,
                                  const nlohmann::json& spec)
{
    return dfm::runDFMForSpec(shape, spec);
}

}  // namespace koocadcam::engine
