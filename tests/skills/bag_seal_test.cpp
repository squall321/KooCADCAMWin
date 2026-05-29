// @lat: [[process/test-strategy#skill round-trip]]
//
// bag_seal — flexible-bag heat-sealing.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. UltrasonicSelectsHornTooling

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bag_seal.hpp"

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

}  // namespace

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillBagSeal, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 4.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::bag_seal::Input in;
    in.seal_type   = "heat";
    in.seal_temp_c = 180.0;
    in.seal_time_s = 1.5;

    auto out = skill::bag_seal::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("bag_seal"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillBagSeal, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 3.0);

    // Unknown seal type.
    skill::bag_seal::Input badType;
    badType.seal_type   = "laser";
    badType.seal_temp_c = 180.0;
    badType.seal_time_s = 1.5;
    {
        auto r = skill::bag_seal::validate(*stock, badType);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::bag_seal::apply(*stock, badType), skill::SkillError);

    // Non-positive seal time.
    skill::bag_seal::Input zeroTime;
    zeroTime.seal_type   = "heat";
    zeroTime.seal_temp_c = 180.0;
    zeroTime.seal_time_s = 0.0;
    {
        auto r = skill::bag_seal::validate(*stock, zeroTime);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-SEAL-TIME"));
    }
    EXPECT_THROW(skill::bag_seal::apply(*stock, zeroTime), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillBagSeal, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 5.0);

    skill::bag_seal::Input in;
    in.seal_type   = "induction";
    in.seal_temp_c = 150.0;
    in.seal_time_s = 2.0;

    auto out = skill::bag_seal::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("bag_seal"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("bag_seal"));
    EXPECT_EQ(out.signature.pattern["seal_type"].get<std::string>(),
              std::string("induction"));
    EXPECT_NEAR(out.signature.pattern["seal_temp_c"].get<double>(),
                150.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["seal_time_s"].get<double>(),
                2.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillBagSeal, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(90.0, 50.0, 3.0);

    skill::bag_seal::Input in;
    in.seal_type   = "heat";
    in.seal_temp_c = 175.0;
    in.seal_time_s = 1.2;

    auto out   = skill::bag_seal::apply(*stock, in);
    auto cands = skill::bag_seal::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["seal_type"].get<std::string>(),
              std::string("heat"));
    EXPECT_NEAR(cands[0].recovered_params["seal_temp_c"].get<double>(),
                175.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::bag_seal::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: ultrasonic modality selects ultrasonic_horn tooling ─────
TEST(SkillBagSeal, UltrasonicSelectsHornTooling)
{
    auto stock = skill::createCuboidStock(70.0, 50.0, 3.0);

    skill::bag_seal::Input in;
    in.seal_type   = "ultrasonic";
    in.seal_temp_c = 80.0;       // ultrasonic is friction-driven; OK
    in.seal_time_s = 0.4;

    auto out = skill::bag_seal::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("ultrasonic_horn"));
}
