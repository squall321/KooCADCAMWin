// @lat: [[process/test-strategy#skill round-trip]]
//
// weaving — 5-case sweep covering identity geometry, DFM rejection of
// non-positive warp/weft counts, signature recording of loom_type +
// warp_count + weft_count + weave_pattern, metadata-replay recognition,
// and fabric_density = warp_count * weft_count arithmetic.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/weaving.hpp"

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
TEST(SkillWeaving, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::weaving::Input in;
    in.loom_type     = "rapier";
    in.warp_count    = 28.0;
    in.weft_count    = 24.0;
    in.weave_pattern = "plain";

    auto out = skill::weaving::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("weaving"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM rejects non-positive warp / weft counts ───────────────────────
TEST(SkillWeaving, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::weaving::Input zeroWarp;
    zeroWarp.warp_count = 0.0; zeroWarp.weft_count = 24.0;
    EXPECT_FALSE(skill::weaving::validate(*stock, zeroWarp).passed);
    EXPECT_THROW(skill::weaving::apply(*stock, zeroWarp), skill::SkillError);

    skill::weaving::Input negWeft;
    negWeft.warp_count = 28.0; negWeft.weft_count = -1.0;
    EXPECT_FALSE(skill::weaving::validate(*stock, negWeft).passed);
    EXPECT_THROW(skill::weaving::apply(*stock, negWeft), skill::SkillError);
}

// ── 3. Signature records kind + loom_type + counts + weave_pattern ───────
TEST(SkillWeaving, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::weaving::Input in;
    in.loom_type     = "air_jet";
    in.warp_count    = 32.0;
    in.weft_count    = 30.0;
    in.weave_pattern = "twill";

    auto out = skill::weaving::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("weaving"));
    EXPECT_EQ(out.signature.pattern["loom_type"].get<std::string>(),
              std::string("air_jet"));
    EXPECT_NEAR(out.signature.pattern["warp_count"].get<double>(),
                32.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["weft_count"].get<double>(),
                30.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["weave_pattern"].get<std::string>(),
              std::string("twill"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("loom"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillWeaving, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::weaving::Input in;
    in.loom_type     = "shuttle";
    in.warp_count    = 26.0;
    in.weft_count    = 22.0;
    in.weave_pattern = "satin";

    auto out = skill::weaving::apply(*stock, in);
    auto cands = skill::weaving::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["loom_type"].get<std::string>(),
              std::string("shuttle"));
    EXPECT_EQ(cands[0].recovered_params["weave_pattern"].get<std::string>(),
              std::string("satin"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::weaving::recognize(raw).empty());
}

// ── 5. fabric_density = warp_count × weft_count (specific assertion) ─────
TEST(SkillWeaving, FabricDensityIsProductOfCounts)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::weaving::Input in;
    in.loom_type     = "water_jet";
    in.warp_count    = 30.0;
    in.weft_count    = 25.0;
    in.weave_pattern = "plain";

    auto out = skill::weaving::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["fabric_density"].get<double>(),
                30.0 * 25.0, 1e-9);

    // Severely unbalanced weft/warp ratio → info finding fires
    skill::weaving::Input unbalanced;
    unbalanced.loom_type     = "rapier";
    unbalanced.warp_count    = 100.0;
    unbalanced.weft_count    = 5.0;        // ratio 0.05, < 0.3
    unbalanced.weave_pattern = "plain";
    auto r = skill::weaving::validate(*stock, unbalanced);
    EXPECT_TRUE(r.passed);
    bool info = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-TEX-BALANCE" && f.severity == "info") {
            info = true; break;
        }
    }
    EXPECT_TRUE(info);
}
