// @lat: [[process/test-strategy#horn_throat_conical_flare]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/horn_throat_conical_flare.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::horn_throat_conical_flare::Input goodInput()
{
    skill::horn_throat_conical_flare::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy       = gp_Pnt(40.0, 40.0, 0.0);
    in.throat_dia_mm   = 25.4;
    in.mouth_dia_mm    = 60.0;
    in.flare_length_mm = 25.0;
    return in;
}
}  // namespace

TEST(SkillHornThroatConicalFlare, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::horn_throat_conical_flare::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // conical flare removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillHornThroatConicalFlare, ValidateRejectsNonExpandingHorn)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    auto in = goodInput();
    in.mouth_dia_mm = 20.0;   // smaller than throat 25.4

    auto r = skill::horn_throat_conical_flare::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FLARE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::horn_throat_conical_flare::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillHornThroatConicalFlare, ValidateRejectsNonPositiveDims)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    auto in = goodInput();
    in.flare_length_mm = 0.0;

    auto r = skill::horn_throat_conical_flare::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillHornThroatConicalFlare, SignatureCompoundFlare)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::horn_throat_conical_flare::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("audio_feature_type", std::string()),
              std::string("conical_horn_flare"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 1);
}

TEST(SkillHornThroatConicalFlare, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::horn_throat_conical_flare::apply(*stock, in);
    auto cands = skill::horn_throat_conical_flare::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
