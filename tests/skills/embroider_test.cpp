// @lat: [[process/test-strategy#skill round-trip]]
//
// embroider skill — 5-case sweep:
//   1. Apply clones geometry unchanged
//   2. Validate rejects empty design & zero stitches
//   3. Signature records kind + design + counts
//   4. Recognize metadata replay
//   5. SPECIFIC: cycle time scales with stitches_count + color changes

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/embroider.hpp"

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

skill::embroider::Input goodInput()
{
    skill::embroider::Input in;
    in.design_id           = "EMB-LOGO-A";
    in.stitches_count      = 8000;
    in.thread_colors_count = 4;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillEmbroider, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::embroider::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("embroider"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillEmbroider, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.design_id = "";    // invalid
    auto r = skill::embroider::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::embroider::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.stitches_count = 0;    // invalid
    auto r2 = skill::embroider::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);

    auto in3 = goodInput();
    in3.thread_colors_count = 0;
    auto r3 = skill::embroider::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillEmbroider, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.design_id           = "EMB-CREST";
    in.stitches_count      = 25000;
    in.thread_colors_count = 7;

    auto out = skill::embroider::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("embroider"));
    EXPECT_EQ(out.signature.pattern["design_id"].get<std::string>(),
              std::string("EMB-CREST"));
    EXPECT_EQ(out.signature.pattern["stitches_count"].get<int>(), 25000);
    EXPECT_EQ(out.signature.pattern["thread_colors_count"].get<int>(), 7);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("multi_head_embroidery_machine"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillEmbroider, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::embroider::apply(*stock, goodInput());

    auto cands = skill::embroider::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["design_id"].get<std::string>(),
              std::string("EMB-LOGO-A"));

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::embroider::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "embroider cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: cycle time scales with stitches + color changes ─────────
TEST(SkillEmbroider, CycleTimeScalesWithStitchesAndColors)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto small = goodInput();
    small.stitches_count      = 1000;
    small.thread_colors_count = 1;
    auto smallOut = skill::embroider::apply(*stock, small);

    auto big = goodInput();
    big.stitches_count      = 50000;
    big.thread_colors_count = 1;
    auto bigOut = skill::embroider::apply(*stock, big);

    auto manyColors = goodInput();
    manyColors.stitches_count      = 1000;
    manyColors.thread_colors_count = 10;
    auto manyColorsOut = skill::embroider::apply(*stock, manyColors);

    EXPECT_LT(smallOut.signature.tooling.est_cycle_time_s,
              bigOut.signature.tooling.est_cycle_time_s);
    EXPECT_LT(smallOut.signature.tooling.est_cycle_time_s,
              manyColorsOut.signature.tooling.est_cycle_time_s);

    // Over-the-limit color count triggers warning but still passes.
    auto over = goodInput();
    over.thread_colors_count = 20;
    auto rOver = skill::embroider::validate(*stock, over);
    EXPECT_TRUE(rOver.passed);
    EXPECT_TRUE(hasFinding(rOver, "DFM-EMB-COLORS"));
}
