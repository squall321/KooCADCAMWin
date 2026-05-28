// @lat: [[process/test-strategy#skill round-trip]]
//
// lapping_op — finest mechanical finish.  Two regimes:
//   - removal_mm < 0.001 mm → geometric no-op (signature-only)
//   - removal_mm ≥ 0.001 mm → delegated face_milling cut
//
// Tests (5):
//   1. Sub-threshold removal returns shape UNCHANGED but records signature.
//   2. Above-threshold removal performs an actual face_milling cut.
//   3. DFM rejects out-of-range removal (< 0.0001, > 0.005).
//   4. Apply carries lap-plate tooling + lapping_compound metadata.
//   5. recognize() returns empty (metadata-only differentiation).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lapping_op.hpp"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

double topZ(const TopoDS_Shape& s)
{
    Bnd_Box b;
    BRepBndLib::AddOptimal(s, b);
    double xMin, yMin, zMin, xMax, yMax, zMax;
    b.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    return zMax;
}

}  // namespace

// ─── 1. Sub-threshold lap leaves shape unchanged + records signature ──────
TEST(SkillLappingOp, SubThresholdIsGeometricNoOp)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());
    const double topBefore = topZ(stock->shape());

    skill::lapping_op::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.removal_mm = 0.0005;            // below 0.001 mm threshold
    in.lapping_compound = "diamond_1um";

    auto out = skill::lapping_op::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Geometry unchanged (sub-threshold).
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-6);
    EXPECT_NEAR(topZ(out.workpiece->shape()), topBefore, 1e-6);

    // But the signature WAS recorded.
    EXPECT_EQ(out.signature.skill_id, std::string("lapping_op"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_TRUE(out.signature.pattern["geometric_no_op"].get<bool>());
    EXPECT_NEAR(out.signature.pattern["removal_mm"].get<double>(), 0.0005, 1e-9);
}

// ─── 2. Above-threshold lap actually cuts (via face_milling) ──────────────
TEST(SkillLappingOp, AboveThresholdRemovesMaterial)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());
    const double topBefore = topZ(stock->shape());

    skill::lapping_op::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.removal_mm = 0.003;            // above 0.001 mm threshold
    in.lapping_compound = "diamond_3um";

    auto out = skill::lapping_op::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Geometry changed: volume reduced by 50*50*0.003 = 7.5 mm³.
    const double diff = volBefore - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, 7.5, 7.5 * 0.10);
    EXPECT_NEAR(topBefore - topZ(out.workpiece->shape()), 0.003, 1e-4);

    EXPECT_FALSE(out.signature.pattern["geometric_no_op"].get<bool>());
    EXPECT_EQ(out.signature.skill_id, std::string("lapping_op"));
}

// ─── 3. DFM rejects out-of-range removal ──────────────────────────────────
TEST(SkillLappingOp, ValidateRejectsOutOfRange)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Below 0.0001
    {
        skill::lapping_op::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.removal_mm = 0.00005;
        auto r = skill::lapping_op::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::lapping_op::apply(*stock, in), skill::SkillError);
    }

    // Above 0.005
    {
        skill::lapping_op::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.removal_mm = 0.01;
        auto r = skill::lapping_op::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::lapping_op::apply(*stock, in), skill::SkillError);
    }

    // Zero
    {
        skill::lapping_op::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.removal_mm = 0.0;
        auto r = skill::lapping_op::validate(*stock, in);
        EXPECT_FALSE(r.passed);
    }
}

// ─── 4. Apply carries lap-plate tooling + compound metadata ───────────────
TEST(SkillLappingOp, ApplyCarriesLapMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::lapping_op::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.removal_mm = 0.001;
    in.lapping_compound = "diamond_3um";

    auto out = skill::lapping_op::apply(*stock, in);

    EXPECT_EQ(out.signature.tooling.tool_type, std::string("lap_plate"));
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("cast_iron"));
    EXPECT_EQ(out.signature.tooling.flute_count, 0);
    // Lapping is the slowest of the finish ops.
    EXPECT_LT(out.signature.tooling.cutting_speed_sfm, 200.0);
    EXPECT_EQ(out.signature.tooling.extra["operation_type"].get<std::string>(),
              std::string("flat_lap"));
    EXPECT_EQ(out.signature.pattern["lapping_compound"].get<std::string>(),
              std::string("diamond_3um"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("lapping_op"));
}

// ─── 5. recognize() returns empty (metadata-only) ─────────────────────────
TEST(SkillLappingOp, RecognizeReturnsEmpty)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::lapping_op::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.removal_mm = 0.002;
    in.lapping_compound = "aluminum_oxide_5um";

    auto out = skill::lapping_op::apply(*stock, in);
    auto cands = skill::lapping_op::recognize(*out.workpiece);

    EXPECT_TRUE(cands.empty())
        << "lapping_op cannot be uniquely recognized from geometry alone";
}
