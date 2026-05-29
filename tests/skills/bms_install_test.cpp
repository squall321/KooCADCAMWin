// @lat: [[process/test-strategy#skill round-trip]]
//
// bms_install skill — 5-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bms_install.hpp"

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

skill::bms_install::Input goodInput()
{
    skill::bms_install::Input in;
    in.bms_id                 = "BMS-EV-001";
    in.channel_count          = 96;
    in.communication_protocol = "CAN";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillBmsInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(200.0, 150.0, 30.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::bms_install::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("bms_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillBmsInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(200.0, 150.0, 30.0);

    auto in = goodInput();
    in.communication_protocol = "MODBUS";
    auto r = skill::bms_install::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-BMS-COMM"));
    EXPECT_THROW(skill::bms_install::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.channel_count = 300;             // > 192
    auto r2 = skill::bms_install::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-BMS-CH"));

    auto in3 = goodInput();
    in3.bms_id = "";
    auto r3 = skill::bms_install::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillBmsInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(200.0, 150.0, 30.0);

    auto in = goodInput();
    in.bms_id                 = "BMS-EV-LIN-007";
    in.channel_count          = 12;
    in.communication_protocol = "LIN";

    auto out = skill::bms_install::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("bms_install"));
    EXPECT_EQ(out.signature.pattern["bms_id"].get<std::string>(),
              std::string("BMS-EV-LIN-007"));
    EXPECT_EQ(out.signature.pattern["channel_count"].get<int>(), 12);
    EXPECT_EQ(out.signature.pattern["communication_protocol"].get<std::string>(),
              std::string("LIN"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillBmsInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(200.0, 150.0, 30.0);
    auto out = skill::bms_install::apply(*stock, goodInput());

    auto cands = skill::bms_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["communication_protocol"]
                  .get<std::string>(),
              std::string("CAN"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::bms_install::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. SPECIFIC: CAN gets 500 kbps, LIN gets 20 kbps in pattern ─────────
TEST(SkillBmsInstall, BusSpeedDependsOnProtocol)
{
    auto stock = skill::createCuboidStock(200.0, 150.0, 30.0);

    auto inCan = goodInput();
    inCan.communication_protocol = "CAN";
    auto outCan = skill::bms_install::apply(*stock, inCan);
    EXPECT_EQ(outCan.signature.pattern["bus_speed_kbps"].get<int>(), 500);

    auto inLin = goodInput();
    inLin.communication_protocol = "LIN";
    auto outLin = skill::bms_install::apply(*stock, inLin);
    EXPECT_EQ(outLin.signature.pattern["bus_speed_kbps"].get<int>(), 20);
}
