// @lat: [[process/test-strategy#skill round-trip]]
//
// knit_apparel skill — 5-case sweep:
//   1. Apply clones geometry unchanged
//   2. Validate rejects empty pattern_id & zero gauge
//   3. Signature records kind + gauge + yarn_color + pattern_id
//   4. Recognize metadata replay
//   5. SPECIFIC: high gauge selects flat-knit, low gauge circular-knit

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/knit_apparel.hpp"

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

skill::knit_apparel::Input goodInput()
{
    skill::knit_apparel::Input in;
    in.gauge      = 12;
    in.yarn_color = "charcoal";
    in.pattern_id = "KNIT-SWEATER-A";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillKnitApparel, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::knit_apparel::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("knit_apparel"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillKnitApparel, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.pattern_id = "";    // invalid
    auto r = skill::knit_apparel::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::knit_apparel::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.gauge = 0;    // invalid
    auto r2 = skill::knit_apparel::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillKnitApparel, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.gauge      = 18;
    in.yarn_color = "cream";
    in.pattern_id = "KNIT-CARDIGAN-B";

    auto out = skill::knit_apparel::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("knit_apparel"));
    EXPECT_EQ(out.signature.pattern["gauge"].get<int>(), 18);
    EXPECT_EQ(out.signature.pattern["yarn_color"].get<std::string>(),
              std::string("cream"));
    EXPECT_EQ(out.signature.pattern["pattern_id"].get<std::string>(),
              std::string("KNIT-CARDIGAN-B"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillKnitApparel, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::knit_apparel::apply(*stock, goodInput());

    auto cands = skill::knit_apparel::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["pattern_id"].get<std::string>(),
              std::string("KNIT-SWEATER-A"));

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::knit_apparel::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "knit_apparel cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: gauge selects machine class ─────────────────────────────
TEST(SkillKnitApparel, GaugeSelectsMachineClass)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto coarse = goodInput();
    coarse.gauge = 7;        // below 18 → circular
    auto coarseOut = skill::knit_apparel::apply(*stock, coarse);
    EXPECT_EQ(coarseOut.signature.tooling.tool_type,
              std::string("circular_knit_machine"));

    auto fine = goodInput();
    fine.gauge = 24;         // ≥ 18 → flat
    auto fineOut = skill::knit_apparel::apply(*stock, fine);
    EXPECT_EQ(fineOut.signature.tooling.tool_type,
              std::string("flat_knit_machine"));

    // Out-of-range gauge yields warning, not block.
    auto huge = goodInput();
    huge.gauge = 50;
    auto rHuge = skill::knit_apparel::validate(*stock, huge);
    EXPECT_TRUE(rHuge.passed);
    EXPECT_TRUE(hasFinding(rHuge, "DFM-KNIT-GAUGE"));
}
