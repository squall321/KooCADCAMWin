// @lat: [[process/test-strategy#helium_escape_valve_seat]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/helium_escape_valve_seat.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::helium_escape_valve_seat::Input goodInput()
{
    skill::helium_escape_valve_seat::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy          = gp_Pnt(20.0, 20.0, 0.0);
    in.valve_bore_dia_mm  = 2.0;
    in.spring_seat_dia_mm = 5.0;
    in.seat_depth_mm      = 3.0;
    in.o_ring_size_key    = "-011";
    return in;
}
}  // namespace

TEST(SkillHeliumEscapeValveSeat, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 12.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::helium_escape_valve_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillHeliumEscapeValveSeat, ValidateRejectsUnknownORing)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 12.0);
    auto in = goodInput();
    in.o_ring_size_key = "-999";

    auto r = skill::helium_escape_valve_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-AS568") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::helium_escape_valve_seat::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillHeliumEscapeValveSeat, ValidateRejectsSeatTooSmall)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 12.0);
    auto in = goodInput();
    in.spring_seat_dia_mm = 1.5;   // <= valve_bore_dia (2.0)

    auto r = skill::helium_escape_valve_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SEAT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillHeliumEscapeValveSeat, SignatureCompoundWatch)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 12.0);
    auto in    = goodInput();
    auto out   = skill::helium_escape_valve_seat::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("helium_escape_valve_seat"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("watch_feature_type", std::string()),
              std::string("helium_escape_valve_seat"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillHeliumEscapeValveSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 12.0);
    auto in    = goodInput();
    auto out   = skill::helium_escape_valve_seat::apply(*stock, in);
    auto cands = skill::helium_escape_valve_seat::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
