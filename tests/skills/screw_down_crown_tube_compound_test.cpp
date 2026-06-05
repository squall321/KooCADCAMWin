// @lat: [[process/test-strategy#screw_down_crown_tube_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/screw_down_crown_tube_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::screw_down_crown_tube_compound::Input goodInput()
{
    skill::screw_down_crown_tube_compound::Input in;
    in.face_id          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    // Bore from the +X side face inward (-X).  Stock is 40x40x12.
    in.bore_origin      = gp_Pnt(40.0, 20.0, 6.0);
    in.bore_dir         = gp_Dir(-1, 0, 0);
    in.tube_bore_dia_mm = 3.0;
    in.thread_size_key  = "M5";
    in.o_ring_size_key  = "-006";
    in.shoulder_dia_mm  = 6.0;
    return in;
}
}  // namespace

TEST(SkillScrewDownCrownTubeCompound, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 12.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::screw_down_crown_tube_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillScrewDownCrownTubeCompound, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 12.0);
    auto in = goodInput();
    in.thread_size_key = "M99";

    auto r = skill::screw_down_crown_tube_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::screw_down_crown_tube_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillScrewDownCrownTubeCompound, ValidateRejectsSmallShoulder)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 12.0);
    auto in = goodInput();
    in.shoulder_dia_mm = 2.0;   // <= tube_bore_dia (3.0)

    auto r = skill::screw_down_crown_tube_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SHOULDER") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillScrewDownCrownTubeCompound, SignatureCompoundWatch)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 12.0);
    auto in    = goodInput();
    auto out   = skill::screw_down_crown_tube_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("screw_down_crown_tube_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("watch_feature_type", std::string()),
              std::string("screw_down_crown_tube"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 5);
}

TEST(SkillScrewDownCrownTubeCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 12.0);
    auto in    = goodInput();
    auto out   = skill::screw_down_crown_tube_compound::apply(*stock, in);
    auto cands = skill::screw_down_crown_tube_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
