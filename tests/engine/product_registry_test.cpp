// @lat: [[engine/feature-watch#JSON Schema]]
//
// Product registry — the unified product-tag-driven build/validate entry point.
// Proves a generic caller builds + DFM-validates ANY product from a tagged or
// key-inferred spec, dispatching to the same result the per-class builder gives.

#include <gtest/gtest.h>

#include "engine/ProductRegistry.hpp"
#include "engine/PhoneFrontModel.hpp"
#include "engine/WatchFrontModel.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace koocadcam;
using namespace koocadcam::engine;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}
}  // namespace

TEST(ProductRegistry, ProductListAndDefaultSpecs)
{
    const auto list = productList();
    EXPECT_NE(std::find(list.begin(), list.end(), "watch"), list.end());
    EXPECT_NE(std::find(list.begin(), list.end(), "phone"), list.end());

    EXPECT_TRUE(defaultSpecForProduct("watch").contains("bezel"));
    EXPECT_TRUE(defaultSpecForProduct("phone").contains("cameras"));
}

TEST(ProductRegistry, ProductFromSpecTagAndInference)
{
    // Explicit tag wins.
    EXPECT_EQ(productFromSpec(nlohmann::json{ { "product", "phone" } }), "phone");
    EXPECT_EQ(productFromSpec(nlohmann::json{ { "product", "watch" } }), "watch");
    // Inference from discriminating keys.
    EXPECT_EQ(productFromSpec(nlohmann::json{ { "cameras", nlohmann::json::array() } }), "phone");
    EXPECT_EQ(productFromSpec(nlohmann::json{ { "crown_cavity", nlohmann::json::object() } }), "watch");
    // Untagged minimal spec defaults to watch.
    EXPECT_EQ(productFromSpec(nlohmann::json::object()), "watch");
}

// productFromGeometry gives a RECOVERED design (a shape with no spec) a product
// identity from its bounding-box shape alone.
TEST(ProductRegistry, ProductFromGeometryInfersFromShape)
{
    std::vector<BuildWarning> w1, w2;
    TopoDS_Shape watch = WatchFrontModel::buildAll(WatchFrontModel::defaultSpec(), w1);
    TopoDS_Shape phone = PhoneFrontModel::buildAll(PhoneFrontModel::defaultSpec(), w2);
    ASSERT_FALSE(watch.IsNull());
    ASSERT_FALSE(phone.IsNull());

    EXPECT_EQ(productFromGeometry(watch), "watch")
        << "a squarish thin disc must be recognised as a watch";
    EXPECT_EQ(productFromGeometry(phone), "phone")
        << "an elongated thin slab must be recognised as a phone";
    // A null shape does not throw.
    EXPECT_EQ(productFromGeometry(TopoDS_Shape{}), "watch");
}

// buildProduct dispatches to the right builder and yields the same geometry as
// the per-class buildAll for that product.
TEST(ProductRegistry, BuildProductMatchesPerClassBuilder)
{
    nlohmann::json watchSpec = WatchFrontModel::defaultSpec();
    nlohmann::json phoneSpec = PhoneFrontModel::defaultSpec();

    std::vector<BuildWarning> w1, w2, w3, w4;
    TopoDS_Shape watchReg = buildProduct(watchSpec, w1);
    TopoDS_Shape watchCls = WatchFrontModel::buildAll(watchSpec, w2);
    TopoDS_Shape phoneReg = buildProduct(phoneSpec, w3);
    TopoDS_Shape phoneCls = PhoneFrontModel::buildAll(phoneSpec, w4);

    ASSERT_FALSE(watchReg.IsNull());
    ASSERT_FALSE(phoneReg.IsNull());
    EXPECT_NEAR(volumeOf(watchReg), volumeOf(watchCls), 1.0)
        << "registry must build the watch identically to WatchFrontModel";
    EXPECT_NEAR(volumeOf(phoneReg), volumeOf(phoneCls), 1.0)
        << "registry must build the phone identically to PhoneFrontModel";
    // The two products are clearly different shapes (sanity).
    EXPECT_GT(std::abs(volumeOf(watchReg) - volumeOf(phoneReg)), 1000.0);
}

// End-to-end: build + DFM-validate a product through the registry alone.
TEST(ProductRegistry, BuildAndValidateThroughRegistry)
{
    nlohmann::json spec = defaultSpecForProduct("phone");
    std::vector<BuildWarning> w;
    TopoDS_Shape shape = buildProduct(spec, w);
    ASSERT_FALSE(shape.IsNull());
    auto report = runDFMForProduct(shape, spec);
    EXPECT_TRUE(report.passed) << "phone default must clear DFM via the registry";
}
