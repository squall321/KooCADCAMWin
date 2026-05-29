// @lat: [[process/test-strategy#skill round-trip]]
//
// recipe_step skill — 5-case sweep covering metadata-only synthesis,
// DFM rejection of bad input, signature recording, recognize replay,
// and a specific assertion (tolerance > target → info finding).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/recipe_step.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

skill::recipe_step::Input goodInput()
{
    skill::recipe_step::Input in;
    in.step_id        = 12;
    in.parameter_name = "pressure_psi";
    in.target_value   = 145.0;
    in.tolerance      = 2.5;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillRecipeStep, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::recipe_step::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("recipe_step"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (empty param name, neg tol, neg id) ────
TEST(SkillRecipeStep, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.parameter_name = "";
    auto r = skill::recipe_step::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::recipe_step::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.tolerance = -1.0;
    auto r2 = skill::recipe_step::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.step_id = -5;
    auto r3 = skill::recipe_step::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + step_id/param_name/target_value/tol ─────
TEST(SkillRecipeStep, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.step_id        = 73;
    in.parameter_name = "temperature_c";
    in.target_value   = 82.0;
    in.tolerance      = 0.5;

    auto out = skill::recipe_step::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("recipe_step"));
    EXPECT_EQ(out.signature.pattern["step_id"].get<int>(), 73);
    EXPECT_EQ(out.signature.pattern["parameter_name"].get<std::string>(),
              std::string("temperature_c"));
    EXPECT_NEAR(out.signature.pattern["target_value"].get<double>(),
                82.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["tolerance"].get<double>(),
                0.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("recipe_executor"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillRecipeStep, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::recipe_step::apply(*stock, goodInput());

    auto cands = skill::recipe_step::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["parameter_name"].get<std::string>(),
              std::string("pressure_psi"));
    EXPECT_EQ(cands[0].recovered_params["step_id"].get<int>(), 12);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::recipe_step::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "recipe_step cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: tolerance > |target| → DFM info finding (still passes) ─
TEST(SkillRecipeStep, LooseToleranceEmitsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::recipe_step::Input in;
    in.step_id        = 5;
    in.parameter_name = "flow_lpm";
    in.target_value   = 10.0;
    in.tolerance      = 25.0;   // > |target|

    auto r = skill::recipe_step::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "loose tolerance is INFO-level, not blocking";
    EXPECT_TRUE(hasFinding(r, "DFM-RECIPE-TOL"));

    // apply() should succeed.
    EXPECT_NO_THROW(skill::recipe_step::apply(*stock, in));
}
