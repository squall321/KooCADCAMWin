// @lat: [[process/test-strategy#step_io]]
// Round-trip: PlaceholderCylinder → StepIO::write → StepIO::read → volume match ±1 %.

#include <gtest/gtest.h>

#include "engine/PlaceholderCylinder.hpp"
#include "io/StepIO.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <filesystem>
#include <cmath>

namespace fs = std::filesystem;

namespace {

double computeVolume(const TopoDS_Shape& shape)
{
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape, props);
    return props.Mass();
}

}  // namespace

TEST(StepIO, RoundTripPreservesVolumeWithin1Percent)
{
    using namespace koocadcam;

    const TopoDS_Shape original = engine::PlaceholderCylinder::make();
    ASSERT_FALSE(original.IsNull());

    const fs::path tmp = fs::temp_directory_path() / "koocadcam_step_io_test.step";
    std::string err;

    ASSERT_TRUE(io::StepIO::write(original, tmp, err)) << err;
    ASSERT_TRUE(fs::exists(tmp));

    auto loaded = io::StepIO::read(tmp, err);
    ASSERT_TRUE(loaded.has_value()) << err;
    ASSERT_FALSE(loaded->IsNull());

    const double vOrig   = computeVolume(original);
    const double vLoaded = computeVolume(*loaded);
    ASSERT_GT(vOrig, 0.0);

    const double relativeDelta = std::abs(vOrig - vLoaded) / vOrig;
    EXPECT_LT(relativeDelta, 0.01)
        << "Volume mismatch: original=" << vOrig << " loaded=" << vLoaded;

    std::error_code ec;
    fs::remove(tmp, ec);  // best-effort cleanup
}

TEST(StepIO, WriteFailsOnNullShape)
{
    const fs::path tmp = fs::temp_directory_path() / "koocadcam_should_not_exist.step";
    std::string err;
    TopoDS_Shape nullShape;
    EXPECT_FALSE(koocadcam::io::StepIO::write(nullShape, tmp, err));
    EXPECT_FALSE(err.empty());
}

TEST(StepIO, ReadFailsOnMissingFile)
{
    const fs::path tmp = fs::temp_directory_path() / "koocadcam_definitely_not_here.step";
    std::error_code ec;
    std::filesystem::remove(tmp, ec);
    std::string err;
    EXPECT_FALSE(koocadcam::io::StepIO::read(tmp, err).has_value());
    EXPECT_FALSE(err.empty());
}
