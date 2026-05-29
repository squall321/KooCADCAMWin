// @lat: [[process/test-strategy#skill round-trip]]
//
// remelt skill — 5-case sweep covering:
//   1. identity geometry on apply
//   2. DFM rejects bad input (non-positive temp; yield_pct out of range)
//   3. signature carries skill kind + melt_temp_c + refining_method + yield_pct
//   4. recognize via metadata replay
//   5. specific: low yield → warning; unknown refining method → info; proceeds

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/remelt.hpp"

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
TEST(SkillRemelt, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::remelt::Input in;
    in.melt_temp_c     = 720.0;
    in.refining_method = "flux";
    in.yield_pct       = 92.0;

    auto out = skill::remelt::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("remelt"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillRemelt, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::remelt::Input badTemp;
    badTemp.melt_temp_c     = -50.0;            // invalid
    badTemp.refining_method = "flux";
    badTemp.yield_pct       = 90.0;
    auto r1 = skill::remelt::validate(*stock, badTemp);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::remelt::apply(*stock, badTemp), skill::SkillError);

    skill::remelt::Input badYieldHi;
    badYieldHi.melt_temp_c     = 700.0;
    badYieldHi.refining_method = "flux";
    badYieldHi.yield_pct       = 120.0;         // > 100 → invalid
    auto r2 = skill::remelt::validate(*stock, badYieldHi);
    EXPECT_FALSE(r2.passed);

    skill::remelt::Input badYieldZero;
    badYieldZero.melt_temp_c     = 700.0;
    badYieldZero.refining_method = "flux";
    badYieldZero.yield_pct       = 0.0;         // = 0 → invalid
    auto r3 = skill::remelt::validate(*stock, badYieldZero);
    EXPECT_FALSE(r3.passed);
}

// ─── 3. Signature records skill kind + key params ────────────────────────
TEST(SkillRemelt, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::remelt::Input in;
    in.melt_temp_c     = 1450.0;
    in.refining_method = "argon_purge";
    in.yield_pct       = 88.0;

    auto out = skill::remelt::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("remelt"));
    EXPECT_NEAR(out.signature.pattern["melt_temp_c"].get<double>(),
                1450.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["refining_method"].get<std::string>(),
              std::string("argon_purge"));
    EXPECT_NEAR(out.signature.pattern["yield_pct"].get<double>(),
                88.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("furnace"));
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillRemelt, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::remelt::Input in;
    in.melt_temp_c     = 660.0;
    in.refining_method = "vacuum";
    in.yield_pct       = 95.0;

    auto out = skill::remelt::apply(*stock, in);

    auto cands = skill::remelt::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["refining_method"].get<std::string>(),
              std::string("vacuum"));

    skill::Workpiece raw(out.workpiece->shape());
    auto empty = skill::remelt::recognize(raw);
    EXPECT_TRUE(empty.empty())
        << "remelt cannot be recognized geometrically";
}

// ─── 5. Specific: low yield → warning; unknown refining → info; proceeds ─
TEST(SkillRemelt, LowYieldWarningAndUnknownRefiningInfo)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::remelt::Input in;
    in.melt_temp_c     = 700.0;
    in.refining_method = "plasma_blast";   // unrecognised → info
    in.yield_pct       = 65.0;             // < 70 → warning

    auto r = skill::remelt::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "warnings/info must not block apply";
    EXPECT_TRUE(hasFinding(r, "DFM-REMELT-YIELD"));
    EXPECT_TRUE(hasFinding(r, "DFM-REMELT-REFINE"));
    EXPECT_NO_THROW(skill::remelt::apply(*stock, in));
}
