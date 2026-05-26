// @lat: [[process/test-strategy#phone]]
// PhoneFrontModel M2-phase-1 integration tests.
// Validates that the parametric primitives layer (extracted in
// refactor 797903b) composes correctly for rectangular geometry.

#include <gtest/gtest.h>

#include "engine/PhoneFrontModel.hpp"
#include "engine/primitives/Bbox.hpp"
#include "io/JsonSpec.hpp"

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

namespace {

fs::path resolveFixture(const char* leaf)
{
    fs::path p = fs::current_path();
    for (int i = 0; i < 6; ++i) {
        fs::path candidate = p / "tests" / "data" / leaf;
        if (fs::exists(candidate)) return candidate;
        p = p.parent_path();
    }
    if (const char* env = std::getenv("KOO_TESTS_DATA_DIR"))
        return fs::path(env) / leaf;
    return {};
}

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// 1. Sample fixture builds a valid B-rep with ~76×160×8 envelope
TEST(PhoneFrontModel, BuildAllSampleProducesValidShape)
{
    using namespace koocadcam;
    const fs::path specPath = resolveFixture("sample_phone.json");
    ASSERT_FALSE(specPath.empty()) << "sample_phone.json not found";

    std::string err;
    auto specOpt = io::JsonSpec::read(specPath, err);
    ASSERT_TRUE(specOpt.has_value()) << err;

    std::vector<engine::BuildWarning> warnings;
    TopoDS_Shape shape = engine::PhoneFrontModel::buildAll(*specOpt, warnings);
    ASSERT_FALSE(shape.IsNull()) << "buildAll returned null shape";

    BRepCheck_Analyzer analyzer(shape);
    EXPECT_TRUE(analyzer.IsValid()) << "shape failed BRepCheck_Analyzer";

    const auto bb = engine::prim::optimalBbox(shape);
    EXPECT_NEAR(bb.dx(),  76.0, 0.5) << "phone width out of tolerance";
    EXPECT_NEAR(bb.dy(), 160.0, 0.5) << "phone height out of tolerance";
    EXPECT_NEAR(bb.dz(),   8.0, 0.5) << "phone thickness out of tolerance";
}

// 2. buildAll is deterministic
TEST(PhoneFrontModel, BuildAllIsDeterministic)
{
    using namespace koocadcam;
    nlohmann::json spec = engine::PhoneFrontModel::defaultSpec();

    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape s1 = engine::PhoneFrontModel::buildAll(spec, w1);
    TopoDS_Shape s2 = engine::PhoneFrontModel::buildAll(spec, w2);
    ASSERT_FALSE(s1.IsNull());
    ASSERT_FALSE(s2.IsNull());

    EXPECT_GT(volumeOf(s1), 0.0);
    EXPECT_NEAR(volumeOf(s1), volumeOf(s2), 1e-6);
}

// 3. Display pocket removes material from the slab
TEST(PhoneFrontModel, DisplayPocketReducesVolume)
{
    using namespace koocadcam;
    nlohmann::json full = engine::PhoneFrontModel::defaultSpec();
    nlohmann::json noPkt = full;
    noPkt.erase("display_pocket");

    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape sFull  = engine::PhoneFrontModel::buildAll(full,  w1);
    TopoDS_Shape sNoPkt = engine::PhoneFrontModel::buildAll(noPkt, w2);
    ASSERT_FALSE(sFull.IsNull());
    ASSERT_FALSE(sNoPkt.IsNull());

    EXPECT_LT(volumeOf(sFull), volumeOf(sNoPkt))
        << "display pocket should remove material";

    // Pocket volume ~ width * height * depth (sharp-rectangle approximation,
    // 25 % tolerance because rounded corners reduce the actual cut).
    const auto& dp = full["display_pocket"];
    const double approx = dp["width_mm"].get<double>()
                        * dp["height_mm"].get<double>()
                        * dp["depth_mm"].get<double>();
    EXPECT_NEAR(volumeOf(sNoPkt) - volumeOf(sFull), approx, approx * 0.25);
}

// 4. Camera holes count affects volume monotonically
TEST(PhoneFrontModel, MoreCamerasRemoveMoreMaterial)
{
    using namespace koocadcam;
    nlohmann::json oneCam = engine::PhoneFrontModel::defaultSpec();
    oneCam["cameras"] = nlohmann::json::array({ oneCam["cameras"][0] });

    nlohmann::json allCams = engine::PhoneFrontModel::defaultSpec();

    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape sOne = engine::PhoneFrontModel::buildAll(oneCam,  w1);
    TopoDS_Shape sAll = engine::PhoneFrontModel::buildAll(allCams, w2);
    ASSERT_FALSE(sOne.IsNull());
    ASSERT_FALSE(sAll.IsNull());

    EXPECT_LT(volumeOf(sAll), volumeOf(sOne))
        << "3 cameras should remove more material than 1";
}

// 5. Missing optional sections pass through (engine should not crash)
TEST(PhoneFrontModel, OptionalSectionsArePassThrough)
{
    using namespace koocadcam;
    nlohmann::json minimal{
        { "schema_version", "0.1.0" },
        { "product_name",   "Bare Slab" },
        { "base", {
            { "width_mm", 70.0 }, { "height_mm", 140.0 },
            { "thickness_mm", 8.0 }, { "initial_corner_r_mm", 5.0 }
        }},
        { "corner_radius", {
            { "r_top_mm", 0.5 }, { "r_side_mm", 0.5 }
        }}
    };

    std::vector<engine::BuildWarning> warnings;
    TopoDS_Shape shape = engine::PhoneFrontModel::buildAll(minimal, warnings);
    ASSERT_FALSE(shape.IsNull()) << "minimal spec must still build";

    const auto bb = engine::prim::optimalBbox(shape);
    EXPECT_NEAR(bb.dx(),  70.0, 0.5);
    EXPECT_NEAR(bb.dy(), 140.0, 0.5);
    EXPECT_NEAR(bb.dz(),   8.0, 0.5);
}
