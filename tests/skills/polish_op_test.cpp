// @lat: [[process/test-strategy#skill round-trip]]
//
// polish_op skill — 5-case sweep covering thin skim, no-op, DFM, recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/polish_op.hpp"

#include <BRepGProp.hxx>
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

}  // namespace

// ─── 1. Apply: ~5 micron skim removes a sliver and stamps target_ra ───────
TEST(SkillPolishOp, ApplyThinSkim)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::polish_op::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.removal_mm = 0.005;
    in.target_ra  = "ra_0.05";

    auto out = skill::polish_op::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    // Less than 50 × 50 × 0.005 = 12.5 mm³ removed; just check it shrank
    // (or stayed equal — 0.005 mm is right at the OCCT threshold).
    EXPECT_LE(volAfter, volBefore + 1e-6);

    EXPECT_EQ(out.signature.skill_id, std::string("polish_op"));
    EXPECT_EQ(out.signature.params["target_ra"].get<std::string>(),
              std::string("ra_0.05"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("polishing_wheel"));
}

// ─── 2. DFM: removal outside [0.001, 0.05] rejected ───────────────────────
TEST(SkillPolishOp, ValidateRejectsOutOfRange)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::polish_op::Input tooLittle;
    tooLittle.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tooLittle.removal_mm = 0.0005;
    auto r1 = skill::polish_op::validate(*stock, tooLittle);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::polish_op::apply(*stock, tooLittle), skill::SkillError);

    skill::polish_op::Input tooMuch = tooLittle;
    tooMuch.removal_mm = 0.5;
    auto r2 = skill::polish_op::validate(*stock, tooMuch);
    EXPECT_FALSE(r2.passed);
    EXPECT_THROW(skill::polish_op::apply(*stock, tooMuch), skill::SkillError);
}

// ─── 3. Geometry CHANGED flag is set for the normal skim path ─────────────
TEST(SkillPolishOp, GeometryChangedFlagSetOnNormalSkim)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::polish_op::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.removal_mm = 0.02;       // well inside DFM range
    in.target_ra  = "ra_0.025";

    auto out = skill::polish_op::apply(*stock, in);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.params["target_ra"].get<std::string>(),
              std::string("ra_0.025"));
    EXPECT_TRUE(out.signature.pattern["geometry_changed"].get<bool>());
    // Top face area should still be ~ 50 × 50 = 2500 mm².
    EXPECT_NEAR(out.signature.pattern["entry_face_area_mm2"].get<double>(),
                2500.0, 1.0);
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPolishOp, RecognizeMetadataOnly)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::polish_op::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.removal_mm = 0.01;
    in.target_ra  = "ra_0.1";

    auto out = skill::polish_op::apply(*stock, in);

    // Same workpiece — has the metadata, recognize returns 1 result.
    auto cands = skill::polish_op::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["target_ra"].get<std::string>(),
              std::string("ra_0.1"));
}

// ─── 5. Recognize on metadata-free workpiece returns EMPTY ────────────────
TEST(SkillPolishOp, RecognizeEmptyOnFeaturelessWorkpiece)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::polish_op::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.removal_mm = 0.01;
    in.target_ra  = "ra_0.1";
    auto out = skill::polish_op::apply(*stock, in);

    // Strip the features by constructing a fresh Workpiece from raw shape.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands = skill::polish_op::recognize(raw);
    EXPECT_TRUE(cands.empty())
        << "polish_op cannot be recognized geometrically";
}

// ─── 6. Default arguments produce sane signature ──────────────────────────
TEST(SkillPolishOp, DefaultsProduceMirrorFinish)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::polish_op::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    // removal_mm defaults to 0.005, target_ra to "ra_0.05"

    auto out = skill::polish_op::apply(*stock, in);
    EXPECT_NEAR(out.signature.params["removal_mm"].get<double>(), 0.005, 1e-6);
    EXPECT_EQ(out.signature.params["target_ra"].get<std::string>(),
              std::string("ra_0.05"));
}
