// @lat: [[process/test-strategy#skill round-trip]]
//
// contour_turn — multi-pass turning along a smooth contour (≥ 5 points).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/contour_turn.hpp"

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

skill::contour_turn::Input goodInput()
{
    // 6-point smooth contour inside a Ø40 × 20 mm stock.
    skill::contour_turn::Input in;
    in.profile = {
        { 2.0,  10.0 },
        { 5.0,  13.0 },
        { 9.0,  15.0 },
        { 12.0, 14.0 },
        { 15.0, 12.0 },
        { 18.0, 10.0 },
    };
    in.pass_count = 3;
    return in;
}

}  // namespace

// ─── 1. apply removes material ────────────────────────────────────────────
TEST(SkillContourTurn, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    const double volBefore = volumeOf(stock->shape());
    auto out = skill::contour_turn::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_LT(volumeOf(out.workpiece->shape()), volBefore);
}

// ─── 2. DFM rejects < 5 polyline points ──────────────────────────────────
TEST(SkillContourTurn, ValidateRejectsTooFewPoints)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    skill::contour_turn::Input in;
    in.profile = {
        { 2.0, 10.0 }, { 8.0, 13.0 }, { 14.0, 12.0 },   // only 3 points
    };
    in.pass_count = 1;
    auto r = skill::contour_turn::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::contour_turn::apply(*stock, in), skill::SkillError);
}

// ─── 3. signature records kind ────────────────────────────────────────────
TEST(SkillContourTurn, SignatureRecordsKind)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    auto out = skill::contour_turn::apply(*stock, goodInput());
    EXPECT_EQ(out.signature.skill_id, std::string("contour_turn"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("contour_turn"));
    EXPECT_EQ(out.signature.pattern["point_count"].get<int>(), 6);
}

// ─── 4. recognize via metadata replay ─────────────────────────────────────
TEST(SkillContourTurn, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    auto out = skill::contour_turn::apply(*stock, goodInput());
    auto cands = skill::contour_turn::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    ASSERT_TRUE(cands[0].recovered_params.contains("profile"));
    EXPECT_EQ(cands[0].recovered_params["profile"].size(), 6u);
}

// ─── 5. specific — pass_count round-trip and ≥ 1 ──────────────────────────
TEST(SkillContourTurn, PassCountRoundTrip)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    auto in = goodInput();
    in.pass_count = 4;
    auto out = skill::contour_turn::apply(*stock, in);
    auto cands = skill::contour_turn::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].recovered_params["pass_count"].get<int>(), 4);

    // Zero passes must be rejected.
    in.pass_count = 0;
    auto r = skill::contour_turn::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}
