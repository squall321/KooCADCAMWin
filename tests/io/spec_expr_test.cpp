// @lat: [[engine/feature-watch#JSON Schema]]
//
// Driven-dimension pre-pass (modeling roadmap #3a).  resolveExpressions turns
// "=expr" string leaves into numbers — references by dotted path, + - * /,
// parentheses, precedence, unary minus, chained refs via fixpoint — while
// leaving plain strings and existing numbers untouched.  The end-to-end test
// proves a driven watch dimension builds identically to its literal value.

#include <gtest/gtest.h>

#include "io/SpecExpr.hpp"
#include "engine/WatchFrontModel.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>

using json = nlohmann::json;
using koocadcam::io::resolveExpressions;

TEST(SpecExpr, ResolvesArithmeticWithPrecedenceAndRefs)
{
    json s = { { "base", { { "d", 44.0 }, { "t", 2.0 } } },
               { "bezel", { { "outer", "= base.d - 2 * base.t" } } } };
    std::string err;
    ASSERT_TRUE(resolveExpressions(s, err)) << err;
    EXPECT_DOUBLE_EQ(s["bezel"]["outer"].get<double>(), 40.0);  // 44 - (2*2)
}

TEST(SpecExpr, Parentheses)
{
    json s = { { "a", 10.0 }, { "b", "= (a + 2) * 3" } };
    std::string err;
    ASSERT_TRUE(resolveExpressions(s, err)) << err;
    EXPECT_DOUBLE_EQ(s["b"].get<double>(), 36.0);
}

TEST(SpecExpr, UnaryMinus)
{
    json s = { { "a", 5.0 }, { "b", "= -a + 1" } };
    std::string err;
    ASSERT_TRUE(resolveExpressions(s, err)) << err;
    EXPECT_DOUBLE_EQ(s["b"].get<double>(), -4.0);
}

TEST(SpecExpr, LeavesPlainStringsAndNumbersUntouched)
{
    json s = { { "form_factor", "round" }, { "product_name", "Watch 44" }, { "x", 5.0 } };
    const json orig = s;
    std::string err;
    ASSERT_TRUE(resolveExpressions(s, err)) << err;
    EXPECT_EQ(s, orig) << "no '=' fields → spec must be byte-identical";
}

TEST(SpecExpr, ChainedReferencesResolveViaFixpoint)
{
    json s = { { "a", 3.0 }, { "b", "= a + 1" }, { "c", "= b * 2" } };
    std::string err;
    ASSERT_TRUE(resolveExpressions(s, err)) << err;
    EXPECT_DOUBLE_EQ(s["b"].get<double>(), 4.0);
    EXPECT_DOUBLE_EQ(s["c"].get<double>(), 8.0);
}

TEST(SpecExpr, UnknownReferenceErrors)
{
    json s = { { "b", "= nope.field + 1" } };
    std::string err;
    EXPECT_FALSE(resolveExpressions(s, err));
    EXPECT_FALSE(err.empty());
}

TEST(SpecExpr, ReferenceCycleErrors)
{
    json s = { { "a", "= b" }, { "b", "= a" } };
    std::string err;
    EXPECT_FALSE(resolveExpressions(s, err));
}

// End-to-end: a watch dimension DRIVEN from another field builds identically to
// the same dimension written as a literal.
TEST(SpecExpr, DrivenWatchDimensionBuildsEquivalentToLiteral)
{
    using namespace koocadcam;
    json literal = engine::WatchFrontModel::defaultSpec();
    ASSERT_TRUE(literal.contains("display_pocket"));
    const double baseD = literal["base"]["diameter_mm"].get<double>();

    json driven = literal;
    literal["display_pocket"]["d_pocket_mm"] = baseD - 8.0;       // explicit value
    driven["display_pocket"]["d_pocket_mm"]  = "= base.diameter_mm - 8";  // driven

    std::vector<engine::BuildWarning> wl, wd;
    TopoDS_Shape sL = engine::WatchFrontModel::buildAll(literal, wl);
    TopoDS_Shape sD = engine::WatchFrontModel::buildAll(driven,  wd);
    ASSERT_FALSE(sL.IsNull());
    ASSERT_FALSE(sD.IsNull());

    GProp_GProps pL, pD;
    BRepGProp::VolumeProperties(sL, pL);
    BRepGProp::VolumeProperties(sD, pD);
    EXPECT_NEAR(pL.Mass(), pD.Mass(), 1.0)
        << "driven dimension must build identically to its resolved literal";
}
