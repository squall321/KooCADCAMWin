// @lat: [[process/test-strategy#skill round-trip]]
//
// fabric_dyeing — 5-case sweep covering identity geometry, DFM rejection of
// fixation_pct out of [0, 100] or non-positive temp_c, signature recording
// of dye_type + fixation_pct + temp_c, metadata-replay recognition, and
// effluent_dye_pct = 100 - fixation_pct arithmetic.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/fabric_dyeing.hpp"

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

}  // namespace

// ── 1. Apply: geometry is COMPLETELY unchanged ───────────────────────────
TEST(SkillFabricDyeing, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::fabric_dyeing::Input in;
    in.dye_type     = "reactive";
    in.fixation_pct = 85.0;
    in.temp_c       = 60.0;

    auto out = skill::fabric_dyeing::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("fabric_dyeing"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM rejects fixation_pct out of range or non-positive temp_c ──────
TEST(SkillFabricDyeing, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fabric_dyeing::Input neg;
    neg.dye_type = "reactive"; neg.fixation_pct = -1.0; neg.temp_c = 60.0;
    EXPECT_FALSE(skill::fabric_dyeing::validate(*stock, neg).passed);
    EXPECT_THROW(skill::fabric_dyeing::apply(*stock, neg), skill::SkillError);

    skill::fabric_dyeing::Input over;
    over.dye_type = "reactive"; over.fixation_pct = 110.0; over.temp_c = 60.0;
    EXPECT_FALSE(skill::fabric_dyeing::validate(*stock, over).passed);

    skill::fabric_dyeing::Input zeroT;
    zeroT.dye_type = "reactive"; zeroT.fixation_pct = 85.0; zeroT.temp_c = 0.0;
    EXPECT_FALSE(skill::fabric_dyeing::validate(*stock, zeroT).passed);
    EXPECT_THROW(skill::fabric_dyeing::apply(*stock, zeroT), skill::SkillError);
}

// ── 3. Signature records kind + dye_type + fixation_pct + temp_c ─────────
TEST(SkillFabricDyeing, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fabric_dyeing::Input in;
    in.dye_type     = "disperse";
    in.fixation_pct = 92.0;
    in.temp_c       = 130.0;

    auto out = skill::fabric_dyeing::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("fabric_dyeing"));
    EXPECT_EQ(out.signature.pattern["dye_type"].get<std::string>(),
              std::string("disperse"));
    EXPECT_NEAR(out.signature.pattern["fixation_pct"].get<double>(),
                92.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["temp_c"].get<double>(), 130.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("dye_bath"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFabricDyeing, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fabric_dyeing::Input in;
    in.dye_type     = "acid";
    in.fixation_pct = 78.0;
    in.temp_c       = 95.0;

    auto out = skill::fabric_dyeing::apply(*stock, in);
    auto cands = skill::fabric_dyeing::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["dye_type"].get<std::string>(),
              std::string("acid"));
    EXPECT_NEAR(cands[0].recovered_params["fixation_pct"].get<double>(),
                78.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["temp_c"].get<double>(), 95.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::fabric_dyeing::recognize(raw).empty());
}

// ── 5. effluent_dye_pct = 100 - fixation_pct (specific assertion) ────────
TEST(SkillFabricDyeing, EffluentIsComplementOfFixation)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fabric_dyeing::Input in;
    in.dye_type     = "reactive";
    in.fixation_pct = 85.0;
    in.temp_c       = 60.0;

    auto out = skill::fabric_dyeing::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["effluent_dye_pct"].get<double>(),
                100.0 - 85.0, 1e-9);

    // Low fixation → DFM-TEX-FIXATION info finding fires
    skill::fabric_dyeing::Input low;
    low.dye_type     = "reactive";
    low.fixation_pct = 50.0;     // < 70
    low.temp_c       = 60.0;
    auto r = skill::fabric_dyeing::validate(*stock, low);
    EXPECT_TRUE(r.passed);
    bool info = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-TEX-FIXATION" && f.severity == "info") {
            info = true; break;
        }
    }
    EXPECT_TRUE(info);
}
