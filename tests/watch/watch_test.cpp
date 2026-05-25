// @lat: [[process/test-strategy#watch]]
// Round-watch first 3 build steps integration test.

#include <gtest/gtest.h>

#include "engine/ProductFrontModel.hpp"
#include "engine/WatchFrontModel.hpp"
#include "io/JsonSpec.hpp"

#include <BRepCheck_Analyzer.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <vector>
#include <cmath>

namespace fs = std::filesystem;

namespace {
fs::path resolveFixture(const char* leaf)
{
    // Look up sample fixture relative to the test source tree.
    // CMake set CWD to the binary dir; walk up to find tests/data/.
    fs::path p = fs::current_path();
    for (int i = 0; i < 6; ++i) {
        fs::path candidate = p / "tests" / "data" / leaf;
        if (fs::exists(candidate)) return candidate;
        p = p.parent_path();
    }
    // Fall back to KOO_TESTS_DATA_DIR env (set by CMake test props).
    if (const char* env = std::getenv("KOO_TESTS_DATA_DIR")) {
        return fs::path(env) / leaf;
    }
    return {};
}
}

TEST(WatchFrontModel, BuildAllSampleRoundWatchValidShape)
{
    using namespace koocadcam;
    const fs::path specPath = resolveFixture("sample_watch.json");
    ASSERT_FALSE(specPath.empty()) << "sample_watch.json not found";

    std::string err;
    auto specOpt = io::JsonSpec::read(specPath, err);
    ASSERT_TRUE(specOpt.has_value()) << err;

    std::vector<engine::BuildWarning> warnings;
    TopoDS_Shape shape = engine::WatchFrontModel::buildAll(*specOpt, warnings);
    ASSERT_FALSE(shape.IsNull()) << "buildAll returned null shape";

    BRepCheck_Analyzer analyzer(shape);
    EXPECT_TRUE(analyzer.IsValid()) << "shape failed BRepCheck_Analyzer";

    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    const double dx = xmax - xmin;
    const double dy = ymax - ymin;
    const double dz = zmax - zmin;
    EXPECT_NEAR(dx, 44.0, 2.0);
    EXPECT_NEAR(dy, 44.0, 2.0);
    EXPECT_NEAR(dz, 10.0, 0.5);
}

TEST(WatchFrontModel, BuildAllIsDeterministic)
{
    using namespace koocadcam;
    nlohmann::json spec = engine::WatchFrontModel::defaultSpec();

    auto volumeOf = [](const TopoDS_Shape& s) {
        GProp_GProps props;
        BRepGProp::VolumeProperties(s, props);
        return props.Mass();
    };

    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape s1 = engine::WatchFrontModel::buildAll(spec, w1);
    TopoDS_Shape s2 = engine::WatchFrontModel::buildAll(spec, w2);
    ASSERT_FALSE(s1.IsNull());
    ASSERT_FALSE(s2.IsNull());
    const double v1 = volumeOf(s1);
    const double v2 = volumeOf(s2);
    EXPECT_GT(v1, 0.0);
    EXPECT_NEAR(v1, v2, 1e-6);  // 1 µm^3 tolerance (way below realistic mm-scale)
}
