#pragma once
// @lat: [[engine/dfm-rules#ProductDFM]]
//
// Product-agnostic DFM rule engine (breakthrough plan B5.4-B5.5).
//
// One shared implementation of the DFM-001..DFM-023 catalog, parameterized
// by a per-product DFMProfile, so watch / phone / future products share one
// rule engine instead of copy-pasting 370 lines per product (the old
// WatchFrontModel TODO said "PhoneFrontModel will get its own copy" —
// this module is what it gets instead).  Thresholds that differ between
// products live in the profile; rule LOGIC lives here once.

#include "DFMReport.hpp"

#include <TopoDS_Shape.hxx>

#include <nlohmann/json_fwd.hpp>

#include <string>

namespace koocadcam::engine::dfm {

struct DFMProfile
{
    std::string product;                        // "watch" | "phone"
    double      minWallMm = 0.40;               // DFM-001 limit
    bool        displayPocketFlatnessRule = false;   // DFM-013 (phone)
    bool        decoRingRule = false;                // DFM-018 (phone)
};

// Canonical profiles.
DFMProfile watchProfile();
DFMProfile phoneProfile();

// Product-identity → DFM profile.  Maps a product tag ("watch"/"phone") to its
// canonical profile; an unknown/empty tag falls back to the watch profile.
// This is the first piece of product identity flowing through the pipeline:
// a generic caller can pick the right rule set from a tag instead of
// hard-coding watchProfile()/phoneProfile() at the call site.
DFMProfile profileForProduct(const std::string& product);

// Run the shared catalog against a built shape + its spec.
DFMReport runProductDFM(const TopoDS_Shape& shape,
                        const nlohmann::json& spec,
                        const DFMProfile& profile);

// Product-aware DFM with NO compile-time product knowledge: resolve the product
// from spec["product"] when present, else INFER it from discriminating spec
// keys (cameras/port_hole → phone; bezel/crown_cavity → watch), then dispatch
// through profileForProduct + runProductDFM.  This lets a recovered/adapted
// design — which carries a spec but not a C++ product type — get the correct
// rule set, the bridge the per-class runDFM() methods now delegate to.
DFMReport runDFMForSpec(const TopoDS_Shape& shape, const nlohmann::json& spec);

}  // namespace koocadcam::engine::dfm
