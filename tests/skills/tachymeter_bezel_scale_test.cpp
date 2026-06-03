// @lat: [[process/test-strategy#compound tachymeter_bezel_scale]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tachymeter_bezel_scale.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::tachymeter_bezel_scale::Input goodInput() {
    skill::tachymeter_bezel_scale::Input in;
    in.case_top_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.case_axis         = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    in.outer_dia_mm      = 38.0;
    in.inner_dia_mm      = 32.0;
    in.height_mm         = 1.0;
    in.marking_depth_mm  = 0.3;
    in.marking_count     = 60;
    in.marking_width_mm  = 0.3;
    return in;
}
}  // namespace

// ─── 1. Apply removes bezel volume + 60 marking volume ────────────────────
TEST(SkillTachymeterBezel, ApplyRemovesBezelAndMarkings)
{
    auto stock = skill::createCylindricalStock(40.0, 6.0);

    auto in = goodInput();
    auto out = skill::tachymeter_bezel_scale::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double rOuter = 19.0;
    const double rInner = 16.0;
    const double bezelVol = M_PI * (rOuter * rOuter - rInner * rInner) * 1.0;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());

    EXPECT_GT(diff, bezelVol * 0.90);
    EXPECT_LT(diff, bezelVol * 1.20);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects bad outer ≤ inner ───────────────────────────────
TEST(SkillTachymeterBezel, ValidateRejectsBadGeom)
{
    auto stock = skill::createCylindricalStock(40.0, 6.0);

    auto in = goodInput();
    in.outer_dia_mm = 30.0;   // < inner_dia_mm
    auto r = skill::tachymeter_bezel_scale::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BEZEL-GEOM") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::tachymeter_bezel_scale::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects bad marking_count ───────────────────────────────
TEST(SkillTachymeterBezel, ValidateRejectsBadMarkingCount)
{
    auto stock = skill::createCylindricalStock(40.0, 6.0);

    auto in = goodInput();
    in.marking_count = 5;   // < 10
    auto r = skill::tachymeter_bezel_scale::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MARKING-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + watch_feature + tachymeter ──────────
TEST(SkillTachymeterBezel, SignatureRecordsCompoundAndWatchFeature)
{
    auto stock = skill::createCylindricalStock(40.0, 6.0);

    auto in = goodInput();
    auto out = skill::tachymeter_bezel_scale::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("tachymeter_bezel_scale"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_watch_feature").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("marking_count").get<int>(), 60);
    EXPECT_EQ(out.signature.pattern.at("scale_kind").get<std::string>(),
              std::string("tachymeter"));
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillTachymeterBezel, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 6.0);

    auto in = goodInput();
    auto out = skill::tachymeter_bezel_scale::apply(*stock, in);

    auto cands = skill::tachymeter_bezel_scale::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("marking_count").get<int>(), 60);
}
