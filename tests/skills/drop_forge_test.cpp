// @lat: [[process/test-strategy#skill round-trip]]
//
// drop_forge — metadata-only impact-die forging.  Geometry is unchanged;
// the signature records hammer_energy_kj × blow_count.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drop_forge.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
bool hasFinding(const skill::DFMReport& r, const std::string& code) {
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}
}  // namespace

// ── 1. ApplyHandlesInput — geometry is NOT changed ──────────────────────
TEST(SkillDropForge, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    const double volBefore = volumeOf(stock->shape());

    skill::drop_forge::Input in;
    in.hammer_energy_kj = 25.0;
    in.blow_count       = 4;

    auto out = skill::drop_forge::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.signature.skill_id, std::string("drop_forge"));
}

// ── 2. ValidateRejectsBadInput — zero energy / zero blows ───────────────
TEST(SkillDropForge, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::drop_forge::Input zeroE;
    zeroE.hammer_energy_kj = 0.0;
    zeroE.blow_count       = 3;
    auto r1 = skill::drop_forge::validate(*stock, zeroE);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::drop_forge::apply(*stock, zeroE), skill::SkillError);

    skill::drop_forge::Input zeroB;
    zeroB.hammer_energy_kj = 20.0;
    zeroB.blow_count       = 0;
    auto r2 = skill::drop_forge::validate(*stock, zeroB);
    EXPECT_FALSE(r2.passed);
    EXPECT_THROW(skill::drop_forge::apply(*stock, zeroB), skill::SkillError);
}

// ── 3. SignatureRecordsKind + energy + blows + total ────────────────────
TEST(SkillDropForge, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::drop_forge::Input in;
    in.hammer_energy_kj = 30.0;
    in.blow_count       = 5;

    auto out = skill::drop_forge::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("drop_forge"));
    EXPECT_NEAR(out.signature.params["hammer_energy_kj"].get<double>(),
                30.0, 1e-9);
    EXPECT_EQ(out.signature.params["blow_count"].get<int>(), 5);
    EXPECT_NEAR(out.signature.pattern["total_energy_kj"].get<double>(),
                150.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 4. RecognizeMetadataReplay — recovers all params ─────────────────────
TEST(SkillDropForge, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::drop_forge::Input in;
    in.hammer_energy_kj = 50.0;
    in.blow_count       = 2;

    auto out = skill::drop_forge::apply(*stock, in);
    auto cands = skill::drop_forge::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["hammer_energy_kj"].get<double>(),
                50.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["blow_count"].get<int>(), 2);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::drop_forge::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ── 5. SPECIFIC: > 20 blows produces DFM-DF-BLOWS info finding ──────────
TEST(SkillDropForge, ManyBlowsEmitsInfo)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::drop_forge::Input in;
    in.hammer_energy_kj = 15.0;
    in.blow_count       = 30;          // > 20 → info

    auto r = skill::drop_forge::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-DF-BLOWS"));
    EXPECT_NO_THROW(skill::drop_forge::apply(*stock, in));
}
