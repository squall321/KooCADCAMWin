// @lat: [[process/test-strategy#skill round-trip]]
//
// sew skill — 5-case sweep:
//   1. Apply clones geometry unchanged
//   2. Validate rejects empty stitch_type & zero SPI
//   3. Signature records kind + stitch_type + SPI + color
//   4. Recognize metadata replay
//   5. SPECIFIC: stitch_type dispatches tooling type correctly

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sew.hpp"

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

skill::sew::Input goodInput()
{
    skill::sew::Input in;
    in.stitch_type     = "lockstitch";
    in.stitches_per_in = 12.0;
    in.thread_color    = "navy";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillSew, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::sew::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("sew"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillSew, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.stitch_type = "";   // invalid
    auto r = skill::sew::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::sew::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.stitches_per_in = 0.0;   // invalid
    auto r2 = skill::sew::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillSew, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.stitch_type     = "overlock";
    in.stitches_per_in = 14.0;
    in.thread_color    = "red";

    auto out = skill::sew::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("sew"));
    EXPECT_EQ(out.signature.pattern["stitch_type"].get<std::string>(),
              std::string("overlock"));
    EXPECT_NEAR(out.signature.pattern["stitches_per_in"].get<double>(),
                14.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["thread_color"].get<std::string>(),
              std::string("red"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSew, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::sew::apply(*stock, goodInput());

    auto cands = skill::sew::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["stitch_type"].get<std::string>(),
              std::string("lockstitch"));

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::sew::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "sew cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: stitch_type dispatches tooling type ─────────────────────
TEST(SkillSew, StitchTypeSelectsToolingType)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto lock = goodInput();
    lock.stitch_type = "lockstitch";
    auto lockOut = skill::sew::apply(*stock, lock);
    EXPECT_EQ(lockOut.signature.tooling.tool_type,
              std::string("industrial_lockstitch"));

    auto over = goodInput();
    over.stitch_type = "overlock";
    auto overOut = skill::sew::apply(*stock, over);
    EXPECT_EQ(overOut.signature.tooling.tool_type,
              std::string("overlock_serger"));

    auto chain = goodInput();
    chain.stitch_type = "chainstitch";
    auto chainOut = skill::sew::apply(*stock, chain);
    EXPECT_EQ(chainOut.signature.tooling.tool_type,
              std::string("chainstitch_machine"));

    // Out-of-range SPI produces warning but does not block.
    auto highSpi = goodInput();
    highSpi.stitches_per_in = 40.0;     // above 30 limit → warning
    auto rHigh = skill::sew::validate(*stock, highSpi);
    EXPECT_TRUE(rHigh.passed);
    EXPECT_TRUE(hasFinding(rHigh, "DFM-SEW-DENSITY"));
}
