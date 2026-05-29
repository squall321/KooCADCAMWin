// @lat: [[process/test-strategy#skill round-trip]]
//
// separator_sortation skill — 5-case sweep covering:
//   1. identity geometry on apply
//   2. DFM rejects bad input (purity_pct outside (0, 100], empty fraction)
//   3. signature carries skill kind + method + target_fraction + purity_pct
//   4. recognize via metadata replay
//   5. specific: low purity emits warning + suboptimal pairing → info

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/separator_sortation.hpp"

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

}  // namespace

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillSeparatorSortation, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::separator_sortation::Input in;
    in.method          = "magnetic";
    in.target_fraction = "ferrous";
    in.purity_pct      = 97.0;

    auto out = skill::separator_sortation::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("separator_sortation"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillSeparatorSortation, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::separator_sortation::Input badPurityHi;
    badPurityHi.method          = "magnetic";
    badPurityHi.target_fraction = "ferrous";
    badPurityHi.purity_pct      = 150.0;     // > 100 → invalid
    auto r1 = skill::separator_sortation::validate(*stock, badPurityHi);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::separator_sortation::apply(*stock, badPurityHi),
                 skill::SkillError);

    skill::separator_sortation::Input badPurityZero;
    badPurityZero.method          = "magnetic";
    badPurityZero.target_fraction = "ferrous";
    badPurityZero.purity_pct      = 0.0;     // = 0 → invalid
    auto r2 = skill::separator_sortation::validate(*stock, badPurityZero);
    EXPECT_FALSE(r2.passed);

    skill::separator_sortation::Input badFrac;
    badFrac.method          = "magnetic";
    badFrac.target_fraction = "";            // empty → invalid
    badFrac.purity_pct      = 95.0;
    auto r3 = skill::separator_sortation::validate(*stock, badFrac);
    EXPECT_FALSE(r3.passed);
}

// ─── 3. Signature records skill kind + key params ────────────────────────
TEST(SkillSeparatorSortation, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::separator_sortation::Input in;
    in.method          = "eddy_current";
    in.target_fraction = "non_ferrous";
    in.purity_pct      = 92.5;

    auto out = skill::separator_sortation::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("separator_sortation"));
    EXPECT_EQ(out.signature.pattern["method"].get<std::string>(),
              std::string("eddy_current"));
    EXPECT_EQ(out.signature.pattern["target_fraction"].get<std::string>(),
              std::string("non_ferrous"));
    EXPECT_NEAR(out.signature.pattern["purity_pct"].get<double>(),
                92.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("separator"));
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillSeparatorSortation, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::separator_sortation::Input in;
    in.method          = "density";
    in.target_fraction = "polymer";
    in.purity_pct      = 93.0;

    auto out = skill::separator_sortation::apply(*stock, in);

    auto cands = skill::separator_sortation::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["target_fraction"].get<std::string>(),
              std::string("polymer"));

    skill::Workpiece raw(out.workpiece->shape());
    auto empty = skill::separator_sortation::recognize(raw);
    EXPECT_TRUE(empty.empty())
        << "separator_sortation cannot be recognized geometrically";
}

// ─── 5. Specific: low purity → warning; suboptimal pairing → info ────────
TEST(SkillSeparatorSortation, LowPurityWarningAndSuboptimalPairInfo)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::separator_sortation::Input low;
    low.method          = "magnetic";
    low.target_fraction = "polymer";   // magnetic + polymer is suboptimal
    low.purity_pct      = 80.0;        // < 90 → warning

    auto r = skill::separator_sortation::validate(*stock, low);
    EXPECT_TRUE(r.passed)
        << "purity warning + pairing info must not block apply";
    EXPECT_TRUE(hasFinding(r, "DFM-SORT-PURITY"));
    EXPECT_TRUE(hasFinding(r, "DFM-SORT-COMPAT"));
    EXPECT_NO_THROW(skill::separator_sortation::apply(*stock, low));
}
