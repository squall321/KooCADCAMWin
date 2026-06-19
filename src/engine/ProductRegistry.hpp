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

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
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

// Infer a product identity from RECOVERED GEOMETRY (no spec available) — the
// recognize-side companion to productFromSpec.  A reverse-engineered design
// arrives as a TopoDS_Shape with no product tag; this gives it a starting
// identity (a user can override) so it can flow through the registry.
//
// Heuristic on the optimal bounding box: a watch is a squarish/round thin disc
// (footprint aspect near 1, modest size); a phone is an elongated thin slab.
// Returns "phone" / "watch"; ambiguous shapes default to "watch".
inline std::string productFromGeometry(const TopoDS_Shape& shape)
{
    if (shape.IsNull()) return "watch";
    Bnd_Box b;
    BRepBndLib::AddOptimal(shape, b);
    if (b.IsVoid()) return "watch";
    double xmin, ymin, zmin, xmax, ymax, zmax;
    b.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    const double dx = xmax - xmin, dy = ymax - ymin;
    const double major = std::max(dx, dy);
    const double minor = std::min(dx, dy);
    const double aspect = (minor > 1e-6) ? (major / minor) : 1.0;

    if (aspect >= 1.5 && major >= 90.0) return "phone";   // elongated, large
    if (aspect < 1.4 && major <= 70.0) return "watch";    // squarish, modest
    return "watch";                                       // ambiguous → default
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
