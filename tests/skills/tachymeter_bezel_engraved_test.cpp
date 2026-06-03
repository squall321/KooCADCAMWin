// @lat: [[process/test-strategy#compound tachymeter_bezel_engraved]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tachymeter_bezel_engraved.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::tachymeter_bezel_engraved::Input goodInput() {
    skill::tachymeter_bezel_engraved::Input in;
    in.case_top_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.case_axis           = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    in.bezel_outer_dia_mm  = 38.0;
    in.bezel_inner_dia_mm  = 32.0;
    in.engrave_depth_mm    = 0.2;
    in.major_count         = 12;
    in.minor_count         = 48;
    in.major_tick_width_mm = 0.4;
    in.minor_tick_width_mm = 0.2;
    return in;
}
}  // namespace

// ─── 1. Apply produces shape with non-trivial face/edge growth ───────────
TEST(SkillTachymeterBezel, ApplyProducesEngravingMarks)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::tachymeter_bezel_engraved::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Some volume must be removed.
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(diff, 0.5);     // > 0.5 mm³ minimum
    // Number of faces should grow because each tick is a rectangular cut.
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects engrave depth out of range ──────────────────────
TEST(SkillTachymeterBezel, ValidateRejectsBadDepth)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.engrave_depth_mm = 1.5;   // > 0.8 mm

    auto r = skill::tachymeter_bezel_engraved::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ENGRAVE-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::tachymeter_bezel_engraved::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects major_count not in standard set ─────────────────
TEST(SkillTachymeterBezel, ValidateRejectsBadMajorCount)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.major_count = 18;   // not in {12, 24, 60}
    in.minor_count = 36;   // would be divisible but major itself invalid

    auto r = skill::tachymeter_bezel_engraved::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MAJOR-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records watch feature type + counts ────────────────────
TEST(SkillTachymeterBezel, SignatureRecordsTachymeter)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::tachymeter_bezel_engraved::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("tachymeter_bezel_engraved"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_watch_feature").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("watch_feature_type").get<std::string>(),
              std::string("tachymeter_scale"));
    EXPECT_EQ(out.signature.pattern.at("major_count").get<int>(), 12);
    EXPECT_EQ(out.signature.pattern.at("minor_count").get<int>(), 48);
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillTachymeterBezel, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::tachymeter_bezel_engraved::apply(*stock, in);

    auto cands = skill::tachymeter_bezel_engraved::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("major_count").get<int>(), 12);
}
