// @lat: [[process/test-strategy#skill round-trip]]
//
// sterile_fill — aseptic filling metadata stamp.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. Iso7AsepticEmitsAnnex1Info

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sterile_fill.hpp"

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
TEST(SkillSterileFill, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::sterile_fill::Input in;
    in.fill_volume_ml = 10.0;
    in.container_type = "vial";
    in.env_class      = "ISO_5";

    auto out = skill::sterile_fill::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillSterileFill, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 3.0);

    // Non-positive fill volume.
    skill::sterile_fill::Input zeroV;
    zeroV.fill_volume_ml = 0.0;
    zeroV.container_type = "vial";
    zeroV.env_class      = "ISO_5";
    {
        auto r = skill::sterile_fill::validate(*stock, zeroV);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::sterile_fill::apply(*stock, zeroV),
                 skill::SkillError);

    // Invalid environment class — aseptic gate.
    skill::sterile_fill::Input badEnv;
    badEnv.fill_volume_ml = 5.0;
    badEnv.container_type = "vial";
    badEnv.env_class      = "ISO_9";       // not allowed
    {
        auto r = skill::sterile_fill::validate(*stock, badEnv);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-SF-ENV"));
    }
    EXPECT_THROW(skill::sterile_fill::apply(*stock, badEnv),
                 skill::SkillError);
}

// ─── 3. Signature records kind + key params ────────────────────────────────
TEST(SkillSterileFill, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::sterile_fill::Input in;
    in.fill_volume_ml = 2.5;
    in.container_type = "syringe";
    in.env_class      = "ISO_5";

    auto out = skill::sterile_fill::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("sterile_fill"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("sterile_fill"));
    EXPECT_NEAR(out.signature.pattern["fill_volume_ml"].get<double>(),
                2.5, 1e-9);
    EXPECT_EQ(out.signature.pattern["container_type"].get<std::string>(),
              std::string("syringe"));
    EXPECT_EQ(out.signature.pattern["env_class"].get<std::string>(),
              std::string("ISO_5"));
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ──────────────────────────────────────────
TEST(SkillSterileFill, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::sterile_fill::Input in;
    in.fill_volume_ml = 1.0;
    in.container_type = "ampoule";
    in.env_class      = "ISO_5";

    auto out   = skill::sterile_fill::apply(*stock, in);
    auto cands = skill::sterile_fill::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["fill_volume_ml"].get<double>(),
                1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["container_type"].get<std::string>(),
              std::string("ampoule"));
    EXPECT_EQ(cands[0].recovered_params["env_class"].get<std::string>(),
              std::string("ISO_5"));

    // Stripped of metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::sterile_fill::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. ISO_7 aseptic emits Annex-1 info, still proceeds ───────────────────
TEST(SkillSterileFill, Iso7AsepticEmitsAnnex1Info)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 2.0);

    skill::sterile_fill::Input in;
    in.fill_volume_ml = 10.0;
    in.container_type = "vial";
    in.env_class      = "ISO_7";       // accepted but info-flagged

    auto r = skill::sterile_fill::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-SF-ENV-CLASS"));

    auto out = skill::sterile_fill::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("aseptic_filler"));
}
