// @lat: [[process/test-strategy#skill round-trip]]
//
// vacuum_pack — evacuated flexible bag.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. WeakVacuumEmitsInfoButProceeds

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/vacuum_pack.hpp"

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
TEST(SkillVacuumPack, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 4.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::vacuum_pack::Input in;
    in.vacuum_mbar = 5.0;
    in.seal_time_s = 2.5;

    auto out = skill::vacuum_pack::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("vacuum_pack"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillVacuumPack, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 3.0);

    // Non-positive vacuum.
    skill::vacuum_pack::Input zeroVac;
    zeroVac.vacuum_mbar = 0.0;
    zeroVac.seal_time_s = 2.0;
    {
        auto r = skill::vacuum_pack::validate(*stock, zeroVac);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::vacuum_pack::apply(*stock, zeroVac), skill::SkillError);

    // Non-positive seal time.
    skill::vacuum_pack::Input zeroTime;
    zeroTime.vacuum_mbar = 5.0;
    zeroTime.seal_time_s = -1.0;
    {
        auto r = skill::vacuum_pack::validate(*stock, zeroTime);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::vacuum_pack::apply(*stock, zeroTime), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillVacuumPack, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 5.0);

    skill::vacuum_pack::Input in;
    in.vacuum_mbar = 3.0;
    in.seal_time_s = 3.5;

    auto out = skill::vacuum_pack::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("vacuum_pack"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("vacuum_pack"));
    EXPECT_NEAR(out.signature.pattern["vacuum_mbar"].get<double>(),
                3.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["seal_time_s"].get<double>(),
                3.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("vacuum_chamber_sealer"));
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillVacuumPack, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 3.0);

    skill::vacuum_pack::Input in;
    in.vacuum_mbar = 1.0;
    in.seal_time_s = 4.0;

    auto out   = skill::vacuum_pack::apply(*stock, in);
    auto cands = skill::vacuum_pack::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["vacuum_mbar"].get<double>(),
                1.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::vacuum_pack::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: weak vacuum (> 100 mbar) is info-only, not error ────────
TEST(SkillVacuumPack, WeakVacuumEmitsInfoButProceeds)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 3.0);

    skill::vacuum_pack::Input weak;
    weak.vacuum_mbar = 250.0;    // weak — typical chamber leaves ~ 5 mbar
    weak.seal_time_s = 2.0;

    auto r = skill::vacuum_pack::validate(*stock, weak);
    EXPECT_TRUE(r.passed) << "weak vacuum should NOT block";
    EXPECT_TRUE(hasFinding(r, "DFM-VAC-WEAK"));

    EXPECT_NO_THROW(skill::vacuum_pack::apply(*stock, weak));
}
