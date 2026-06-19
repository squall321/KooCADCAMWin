// @lat: [[engine/dfm-rules#ProductDFM]]
//
// Product-identity bridge (multi-product frontier, first slice).  Until now the
// DFM profile was hard-coded at the call site (WatchFrontModel->watchProfile,
// PhoneFrontModel->phoneProfile).  profileForProduct + runDFMForSpec let a
// design pick its rule set from a product TAG (or inferred from spec keys), so a
// recovered/adapted design — which carries a spec but not a C++ product type —
// gets the right DFM.  These tests prove the tag/inference dispatch resolves to
// the same rule set the per-class methods used.

#include <gtest/gtest.h>

#include "engine/PhoneFrontModel.hpp"
#include "engine/WatchFrontModel.hpp"
#include "engine/dfm/ProductDFM.hpp"

#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>

#include <vector>

using namespace koocadcam;
using namespace koocadcam::engine::dfm;

TEST(ProductIdentity, ProfileForProductMapsTagToProfile)
{
    EXPECT_EQ(profileForProduct("watch").product, "watch");
    EXPECT_EQ(profileForProduct("phone").product, "phone");
    EXPECT_TRUE(profileForProduct("phone").decoRingRule);
    EXPECT_FALSE(profileForProduct("watch").decoRingRule);
    // Unknown / empty tag falls back to the watch profile (never throws).
    EXPECT_EQ(profileForProduct("nonsense").product, "watch");
    EXPECT_EQ(profileForProduct("").product, "watch");
}

// An explicit product tag drives the dispatch: runDFMForSpec on the tagged spec
// yields the same report as calling runProductDFM with that product's profile.
TEST(ProductIdentity, RunDFMForSpecDispatchesByExplicitTag)
{
    nlohmann::json phoneSpec = engine::PhoneFrontModel::defaultSpec();
    ASSERT_EQ(phoneSpec.value("product", std::string{}), "phone");

    std::vector<engine::BuildWarning> w;
    TopoDS_Shape shape = engine::PhoneFrontModel::buildAll(phoneSpec, w);
    ASSERT_FALSE(shape.IsNull());

    const engine::DFMReport viaBridge  = runDFMForSpec(shape, phoneSpec);
    const engine::DFMReport viaProfile = runProductDFM(shape, phoneSpec, phoneProfile());
    EXPECT_EQ(viaBridge.passed, viaProfile.passed);
    EXPECT_EQ(viaBridge.findings.size(), viaProfile.findings.size())
        << "tag dispatch must select the phone rule set";
}

// With NO explicit tag, the product is inferred from discriminating spec keys.
TEST(ProductIdentity, RunDFMForSpecInfersProductFromKeys)
{
    nlohmann::json phoneSpec = engine::PhoneFrontModel::defaultSpec();
    nlohmann::json watchSpec = engine::WatchFrontModel::defaultSpec();
    phoneSpec.erase("product");   // force inference (cameras/port → phone)
    watchSpec.erase("product");   // force inference (bezel/crown/lugs → watch)

    std::vector<engine::BuildWarning> wp, ww;
    TopoDS_Shape phoneShape = engine::PhoneFrontModel::buildAll(phoneSpec, wp);
    TopoDS_Shape watchShape = engine::WatchFrontModel::buildAll(watchSpec, ww);
    ASSERT_FALSE(phoneShape.IsNull());
    ASSERT_FALSE(watchShape.IsNull());

    // Inferred dispatch must match the explicit profile for each product.
    EXPECT_EQ(runDFMForSpec(phoneShape, phoneSpec).findings.size(),
              runProductDFM(phoneShape, phoneSpec, phoneProfile()).findings.size())
        << "cameras/port keys must infer the phone profile";
    EXPECT_EQ(runDFMForSpec(watchShape, watchSpec).findings.size(),
              runProductDFM(watchShape, watchSpec, watchProfile()).findings.size())
        << "bezel/crown/lugs keys must infer the watch profile";
}
