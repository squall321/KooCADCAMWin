// @lat: [[process/test-strategy#skill round-trip]]
//
// knitting — 5-case sweep covering identity geometry, DFM rejection of
// non-positive gauge, signature recording of gauge + yarn_type +
// stitch_pattern, metadata-replay recognition, and stitches_per_cm =
// gauge / 2.54 arithmetic.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/knitting.hpp"

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
TEST(SkillKnitting, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::knitting::Input in;
    in.gauge          = 14.0;
    in.yarn_type      = "cotton";
    in.stitch_pattern = "jersey";

    auto out = skill::knitting::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("knitting"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM rejects non-positive gauge ────────────────────────────────────
TEST(SkillKnitting, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::knitting::Input zeroGauge;
    zeroGauge.gauge = 0.0;
    EXPECT_FALSE(skill::knitting::validate(*stock, zeroGauge).passed);
    EXPECT_THROW(skill::knitting::apply(*stock, zeroGauge), skill::SkillError);

    skill::knitting::Input negGauge;
    negGauge.gauge = -5.0;
    EXPECT_FALSE(skill::knitting::validate(*stock, negGauge).passed);
    EXPECT_THROW(skill::knitting::apply(*stock, negGauge), skill::SkillError);
}

// ── 3. Signature records kind + gauge + yarn_type + stitch_pattern ───────
TEST(SkillKnitting, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::knitting::Input in;
    in.gauge          = 18.0;
    in.yarn_type      = "wool";
    in.stitch_pattern = "rib";

    auto out = skill::knitting::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("knitting"));
    EXPECT_NEAR(out.signature.pattern["gauge"].get<double>(), 18.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["yarn_type"].get<std::string>(),
              std::string("wool"));
    EXPECT_EQ(out.signature.pattern["stitch_pattern"].get<std::string>(),
              std::string("rib"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("knitting_machine"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillKnitting, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::knitting::Input in;
    in.gauge          = 12.0;
    in.yarn_type      = "polyester";
    in.stitch_pattern = "interlock";

    auto out = skill::knitting::apply(*stock, in);
    auto cands = skill::knitting::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["gauge"].get<double>(), 12.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["yarn_type"].get<std::string>(),
              std::string("polyester"));
    EXPECT_EQ(cands[0].recovered_params["stitch_pattern"].get<std::string>(),
              std::string("interlock"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::knitting::recognize(raw).empty());
}

// ── 5. stitches_per_cm derived from gauge (specific assertion) ───────────
TEST(SkillKnitting, StitchesPerCmDerivedFromGauge)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::knitting::Input in;
    in.gauge          = 14.0;
    in.yarn_type      = "cotton";
    in.stitch_pattern = "jersey";

    auto out = skill::knitting::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["stitches_per_cm"].get<double>(),
                14.0 / 2.54, 1e-9);

    // Gauge out of typical industrial range → info finding fires
    skill::knitting::Input coarse;
    coarse.gauge          = 80.0;       // > 60
    coarse.yarn_type      = "cotton";
    coarse.stitch_pattern = "jersey";
    auto r = skill::knitting::validate(*stock, coarse);
    EXPECT_TRUE(r.passed);
    bool info = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-TEX-GAUGE" && f.severity == "info") {
            info = true; break;
        }
    }
    EXPECT_TRUE(info);
}
