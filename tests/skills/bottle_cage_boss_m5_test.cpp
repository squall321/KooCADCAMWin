// @lat: [[process/test-strategy#bottle_cage_boss_m5]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bottle_cage_boss_m5.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::bottle_cage_boss_m5::Input goodInput()
{
    skill::bottle_cage_boss_m5::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.row_origin        = gp_Pnt(20.0, 20.0, 0.0);
    in.boss_spacing_mm   = 64.0;
    in.thread_key        = "M5";
    in.rivnut_seat_dia_mm = 8.0;
    return in;
}
}  // namespace

TEST(SkillBottleCageBossM5, ApplyRemovesMaterial)
{
    // Frame tube panel: 110 x 40 x 6 mm; bosses at x=20 and x=84.
    auto stock = skill::createCuboidStock(110.0, 40.0, 6.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::bottle_cage_boss_m5::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // 2 seats + 2 pilots removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillBottleCageBossM5, ValidateRejectsBadSpacing)
{
    auto stock = skill::createCuboidStock(110.0, 40.0, 6.0);
    auto in = goodInput();
    in.boss_spacing_mm = 50.0;   // not 64 mm

    auto r = skill::bottle_cage_boss_m5::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPACING") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::bottle_cage_boss_m5::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillBottleCageBossM5, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(110.0, 40.0, 6.0);
    auto in = goodInput();
    in.thread_key = "M99";   // not in central table

    auto r = skill::bottle_cage_boss_m5::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillBottleCageBossM5, SignatureCompoundBossPair)
{
    auto stock = skill::createCuboidStock(110.0, 40.0, 6.0);
    auto in    = goodInput();
    auto out   = skill::bottle_cage_boss_m5::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("bottle_cage_boss_m5"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("bicycle_feature_type", std::string()),
              std::string("bottle_cage_boss_pair_m5"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 4);
}

TEST(SkillBottleCageBossM5, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(110.0, 40.0, 6.0);
    auto in    = goodInput();
    auto out   = skill::bottle_cage_boss_m5::apply(*stock, in);
    auto cands = skill::bottle_cage_boss_m5::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
