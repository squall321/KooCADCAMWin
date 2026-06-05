// @lat: [[process/test-strategy#fuel_tank_boss_threaded]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/fuel_tank_boss_threaded.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::fuel_tank_boss_threaded::Input goodInput()
{
    skill::fuel_tank_boss_threaded::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy       = gp_Pnt(40.0, 40.0, 0.0);
    in.boss_dia_mm     = 30.0;
    in.boss_height_mm  = 8.0;
    in.thread_key      = "M12";
    in.o_ring_size_key = "-016";
    return in;
}
}  // namespace

TEST(SkillFuelTankBossThreaded, ApplyAddsMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::fuel_tank_boss_threaded::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v1 - v0, 0.0);   // net additive (boss > bore + groove)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillFuelTankBossThreaded, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    auto in = goodInput();
    in.thread_key = "M99";

    auto r = skill::fuel_tank_boss_threaded::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::fuel_tank_boss_threaded::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillFuelTankBossThreaded, ValidateRejectsUnknownORing)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    auto in = goodInput();
    in.o_ring_size_key = "-999";

    auto r = skill::fuel_tank_boss_threaded::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ORING") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillFuelTankBossThreaded, SignatureCompound)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::fuel_tank_boss_threaded::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("aerostruct_feature_type", std::string()),
              std::string("fuel_tank_boss_threaded"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
    EXPECT_GT(out.signature.pattern.value("derived_volume_added_mm3", 0.0), 0.0);
}

TEST(SkillFuelTankBossThreaded, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::fuel_tank_boss_threaded::apply(*stock, in);
    auto cands = skill::fuel_tank_boss_threaded::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
