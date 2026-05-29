// @lat: [[process/test-strategy#skill round-trip]]
//
// wood_joinery skill — metadata-only joint record.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/wood_joinery.hpp"

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

// ─── 1. Apply handles input: geometry unchanged, signature added ─────────
TEST(SkillWoodJoinery, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(150.0, 50.0, 20.0, "oak");
    const double volBefore = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::wood_joinery::Input in;
    in.joint_type   = "mortise_tenon";
    in.length_mm    = 30.0;
    in.width_mm     = 12.0;
    in.thickness_mm = 10.0;

    auto out = skill::wood_joinery::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Geometry preserved exactly.
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("wood_joinery"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input (unknown joint, zero dims) ────────────
TEST(SkillWoodJoinery, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(150.0, 50.0, 20.0);

    // Unknown joint type → DFM-JOINT-TYPE.
    skill::wood_joinery::Input bad;
    bad.joint_type   = "puzzle_lock";   // unsupported
    bad.length_mm    = 30.0;
    bad.width_mm     = 12.0;
    bad.thickness_mm = 10.0;
    auto r1 = skill::wood_joinery::validate(*stock, bad);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-JOINT-TYPE"));
    EXPECT_THROW(skill::wood_joinery::apply(*stock, bad), skill::SkillError);

    // Zero length → DFM-JOINT-LEN.
    skill::wood_joinery::Input zero;
    zero.joint_type   = "mortise_tenon";
    zero.length_mm    = 0.0;
    zero.width_mm     = 12.0;
    zero.thickness_mm = 10.0;
    auto r2 = skill::wood_joinery::validate(*stock, zero);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-JOINT-LEN"));

    // Zero thickness → DFM-JOINT-THK.
    skill::wood_joinery::Input thinThk;
    thinThk.joint_type   = "dovetail";
    thinThk.length_mm    = 30.0;
    thinThk.width_mm     = 12.0;
    thinThk.thickness_mm = -1.0;
    auto r3 = skill::wood_joinery::validate(*stock, thinThk);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-JOINT-THK"));
}

// ─── 3. Signature records kind + joint_type + dimensions ─────────────────
TEST(SkillWoodJoinery, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(150.0, 50.0, 20.0);

    skill::wood_joinery::Input in;
    in.joint_type   = "dovetail";
    in.length_mm    = 25.0;
    in.width_mm     = 15.0;
    in.thickness_mm = 12.0;

    auto out = skill::wood_joinery::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), std::string("wood_joinery"));
    EXPECT_EQ(out.signature.pattern["joint_type"].get<std::string>(), std::string("dovetail"));
    EXPECT_NEAR(out.signature.pattern["length_mm"].get<double>(), 25.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["width_mm"].get<double>(), 15.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["thickness_mm"].get<double>(), 12.0, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("wood_joinery_fixture"));
}

// ─── 4. Recognize via metadata replay (no geometric fallback) ────────────
TEST(SkillWoodJoinery, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(150.0, 50.0, 20.0);

    skill::wood_joinery::Input in;
    in.joint_type   = "finger";
    in.length_mm    = 20.0;
    in.width_mm     = 10.0;
    in.thickness_mm = 6.0;
    auto out = skill::wood_joinery::apply(*stock, in);

    auto cands = skill::wood_joinery::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["joint_type"].get<std::string>(), std::string("finger"));
    EXPECT_NEAR(cands[0].recovered_params["length_mm"].get<double>(), 20.0, 1e-6);

    // Stripped workpiece → no candidate (joinery is purely metadata).
    skill::Workpiece bare(out.workpiece->shape(), out.workpiece->material());
    auto cands2 = skill::wood_joinery::recognize(bare);
    EXPECT_TRUE(cands2.empty())
        << "wood_joinery is unrecognizable without metadata";
}

// ─── 5. Slender tenon emits info-level ratio finding but still passes ────
TEST(SkillWoodJoinery, SlenderTenonRatioIsInfoNotError)
{
    auto stock = skill::createCuboidStock(200.0, 50.0, 20.0);

    skill::wood_joinery::Input slender;
    slender.joint_type   = "mortise_tenon";
    slender.length_mm    = 60.0;     // long
    slender.width_mm     = 12.0;
    slender.thickness_mm = 6.0;      // thin → ratio = 10 > 5

    auto r = skill::wood_joinery::validate(*stock, slender);
    EXPECT_TRUE(r.passed) << "ratio finding is info, not error";
    EXPECT_TRUE(hasFinding(r, "DFM-JOINT-RATIO"));
    EXPECT_NO_THROW(skill::wood_joinery::apply(*stock, slender));
}
