// @lat: [[process/test-strategy#skill round-trip]]
//
// form_turn — turning with an arbitrary radial profile (polyline revolution).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/form_turn.hpp"

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

skill::form_turn::Input goodInput()
{
    // 4-point bell-shaped profile inside a 40 mm stock, length 20 mm.
    skill::form_turn::Input in;
    in.profile = {
        { 2.0,  10.0 },
        { 7.0,  14.0 },
        { 13.0, 14.0 },
        { 18.0, 10.0 },
    };
    return in;
}

}  // namespace

// ─── 1. apply removes material ────────────────────────────────────────────
TEST(SkillFormTurn, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    const double volBefore = volumeOf(stock->shape());

    auto out = skill::form_turn::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_LT(volumeOf(out.workpiece->shape()), volBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("form_turn"));
}

// ─── 2. DFM rejects non-monotonic z ───────────────────────────────────────
TEST(SkillFormTurn, ValidateRejectsNonMonotonicProfile)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::form_turn::Input in;
    in.profile = {
        { 2.0,  10.0 },
        { 8.0,  12.0 },
        { 5.0,  11.0 },     // z decreased — invalid
        { 18.0, 10.0 },
    };
    auto r = skill::form_turn::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::form_turn::apply(*stock, in), skill::SkillError);
}

// ─── 3. signature records kind ────────────────────────────────────────────
TEST(SkillFormTurn, SignatureRecordsKind)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    auto out = skill::form_turn::apply(*stock, goodInput());
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("form_turn"));
    EXPECT_EQ(out.signature.pattern["point_count"].get<int>(), 4);
}

// ─── 4. recognize via metadata replay ────────────────────────────────────
TEST(SkillFormTurn, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    auto out = skill::form_turn::apply(*stock, goodInput());

    auto cands = skill::form_turn::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    ASSERT_TRUE(cands[0].recovered_params.contains("profile"));
    EXPECT_EQ(cands[0].recovered_params["profile"].size(), 4u);
}

// ─── 5. specific assertion — profile waypoints round-trip ────────────────
TEST(SkillFormTurn, RecoveredProfileMatchesInput)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    auto in = goodInput();
    auto out = skill::form_turn::apply(*stock, in);

    auto cands = skill::form_turn::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    const auto& prof = cands[0].recovered_params["profile"];
    ASSERT_EQ(prof.size(), in.profile.size());
    for (size_t i = 0; i < in.profile.size(); ++i) {
        EXPECT_NEAR(prof[i]["z_mm"].get<double>(), in.profile[i].z_mm, 1e-6);
        EXPECT_NEAR(prof[i]["r_mm"].get<double>(), in.profile[i].r_mm, 1e-6);
    }
}
