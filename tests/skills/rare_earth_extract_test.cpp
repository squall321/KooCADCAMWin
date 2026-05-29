// @lat: [[process/test-strategy#skill round-trip]]
//
// rare_earth_extract skill — 5-case sweep covering identity-geometry
// synthesis, target+feed+recovery DFM gates, metadata signature replay,
// and the recovered_kg = feed_kg * recovery_pct / 100 specific assertion.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rare_earth_extract.hpp"

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
TEST(SkillRareEarthExtract, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::rare_earth_extract::Input in;
    in.feed_kg        = 25.0;
    in.element_target = "Nd";
    in.recovery_pct   = 80.0;

    auto out = skill::rare_earth_extract::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("rare_earth_extract"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: empty target / zero feed / out-of-range recovery rejected ───
TEST(SkillRareEarthExtract, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::rare_earth_extract::Input bad;
    bad.feed_kg        = 10.0;
    bad.element_target = "";              // invalid
    bad.recovery_pct   = 75.0;
    auto r = skill::rare_earth_extract::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::rare_earth_extract::apply(*stock, bad), skill::SkillError);

    bad.element_target = "Nd";
    bad.feed_kg        = 0.0;             // invalid
    EXPECT_FALSE(skill::rare_earth_extract::validate(*stock, bad).passed);

    bad.feed_kg      = 10.0;
    bad.recovery_pct = 0.0;               // invalid (must be > 0)
    EXPECT_FALSE(skill::rare_earth_extract::validate(*stock, bad).passed);

    bad.recovery_pct = 200.0;             // invalid (> 100)
    EXPECT_FALSE(skill::rare_earth_extract::validate(*stock, bad).passed);
}

// ─── 3. Signature records kind + feed + target + recovery ────────────────
TEST(SkillRareEarthExtract, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::rare_earth_extract::Input in;
    in.feed_kg        = 30.0;
    in.element_target = "Dy";
    in.recovery_pct   = 72.0;

    auto out = skill::rare_earth_extract::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("rare_earth_extract"));
    EXPECT_NEAR(out.signature.pattern["feed_kg"].get<double>(),      30.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["element_target"].get<std::string>(),
              std::string("Dy"));
    EXPECT_NEAR(out.signature.pattern["recovery_pct"].get<double>(), 72.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("solvent_extraction_train"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillRareEarthExtract, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::rare_earth_extract::Input in;
    in.feed_kg        = 12.0;
    in.element_target = "Pr";
    in.recovery_pct   = 65.0;

    auto out = skill::rare_earth_extract::apply(*stock, in);
    auto cands = skill::rare_earth_extract::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["element_target"].get<std::string>(),
              std::string("Pr"));
    EXPECT_NEAR(cands[0].recovered_params["feed_kg"].get<double>(), 12.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::rare_earth_extract::recognize(raw).empty())
        << "rare_earth_extract cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: recovered_kg formula + unknown target → info only ──────
TEST(SkillRareEarthExtract, RecoveredKgFormulaAndUnknownTargetIsInfo)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::rare_earth_extract::Input in;
    in.feed_kg        = 50.0;
    in.element_target = "Nd";
    in.recovery_pct   = 60.0;

    auto out = skill::rare_earth_extract::apply(*stock, in);
    // 50 kg × 60 % = 30 kg recovered
    EXPECT_NEAR(out.signature.pattern["recovered_kg"].get<double>(),
                30.0, 1e-9);

    // Unknown element target → info, not error.
    skill::rare_earth_extract::Input exotic;
    exotic.feed_kg        = 10.0;
    exotic.element_target = "Tb";          // not in {Nd, Dy, Pr}
    exotic.recovery_pct   = 50.0;
    auto r = skill::rare_earth_extract::validate(*stock, exotic);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-REE-TARGET"));
    EXPECT_NO_THROW(skill::rare_earth_extract::apply(*stock, exotic));

    // Excessive recovery → info, not error.
    skill::rare_earth_extract::Input optimistic;
    optimistic.feed_kg        = 5.0;
    optimistic.element_target = "Nd";
    optimistic.recovery_pct   = 95.0;      // > 90 info threshold
    auto r2 = skill::rare_earth_extract::validate(*stock, optimistic);
    EXPECT_TRUE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-REE-RECOVERY"));
}
