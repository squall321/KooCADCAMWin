// @lat: [[process/test-strategy#fastener_countersink_array_nas]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/fastener_countersink_array_nas.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::fastener_countersink_array_nas::Input goodInput()
{
    skill::fastener_countersink_array_nas::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.row_origin      = gp_Pnt(15.0, 30.0, 0.0);
    in.fastener_dia_mm = 4.0;
    in.csk_angle_deg   = 100.0;
    in.count           = 4;
    in.pitch_mm        = 12.0;
    return in;
}
}  // namespace

TEST(SkillFastenerCountersinkArrayNas, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 6.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::fastener_countersink_array_nas::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillFastenerCountersinkArrayNas, ValidateRejectsBadAngle)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 6.0);
    auto in = goodInput();
    in.csk_angle_deg = 90.0;   // not in {82, 100, 120}

    auto r = skill::fastener_countersink_array_nas::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ANGLE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::fastener_countersink_array_nas::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillFastenerCountersinkArrayNas, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 6.0);
    auto in = goodInput();
    in.count = 0;

    auto r = skill::fastener_countersink_array_nas::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillFastenerCountersinkArrayNas, SignatureCompound)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 6.0);
    auto in    = goodInput();
    auto out   = skill::fastener_countersink_array_nas::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("aerostruct_feature_type", std::string()),
              std::string("fastener_countersink_array_nas"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 4 * 2);
}

TEST(SkillFastenerCountersinkArrayNas, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 6.0);
    auto in    = goodInput();
    auto out   = skill::fastener_countersink_array_nas::apply(*stock, in);
    auto cands = skill::fastener_countersink_array_nas::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
