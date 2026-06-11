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

// Run the shared catalog against a built shape + its spec.
DFMReport runProductDFM(const TopoDS_Shape& shape,
                        const nlohmann::json& spec,
                        const DFMProfile& profile);

}  // namespace koocadcam::engine::dfm
