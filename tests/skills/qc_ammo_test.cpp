// @lat: [[process/test-strategy#skill round-trip]]
//
// qc_ammo skill — small-arms final QC inspection no-op sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/qc_ammo.hpp"

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

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillQcAmmo, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::qc_ammo::Input in;
    in.cartridge_oal_mm = 29.69;
    in.accept_count     = 100;
    in.reject_count     = 0;

    auto out = skill::qc_ammo::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. ValidateRejectsBadInput: zero sample → error ─────────────────────
TEST(SkillQcAmmo, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::qc_ammo::Input in;
    in.cartridge_oal_mm = 29.69;
    in.accept_count     = 0;
    in.reject_count     = 0;          // empty sample

    auto r = skill::qc_ammo::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::qc_ammo::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillQcAmmo, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::qc_ammo::Input in;
    in.cartridge_oal_mm = 57.4;        // .308 Win
    in.accept_count     = 480;
    in.reject_count     = 20;

    auto out = skill::qc_ammo::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("qc_ammo"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("qc_ammo"));
    EXPECT_NEAR(out.signature.pattern["cartridge_oal_mm"].get<double>(),
                57.4, 1e-6);
    EXPECT_EQ(out.signature.pattern["accept_count"].get<int>(), 480);
    EXPECT_EQ(out.signature.pattern["reject_count"].get<int>(), 20);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillQcAmmo, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::qc_ammo::Input in;
    in.cartridge_oal_mm = 29.69;
    in.accept_count     = 99;
    in.reject_count     = 1;

    auto out  = skill::qc_ammo::apply(*stock, in);
    auto cands = skill::qc_ammo::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["accept_count"].get<int>(), 99);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::qc_ammo::recognize(raw).empty());
}

// ─── 5. Skill-specific: yield_pct computed and low-yield info ────────────
TEST(SkillQcAmmo, YieldPctComputedAndLowYieldInfo)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::qc_ammo::Input in;
    in.cartridge_oal_mm = 29.69;
    in.accept_count     = 80;
    in.reject_count     = 20;         // 80% yield → info

    auto r = skill::qc_ammo::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-AMMO-YIELD"));

    auto out = skill::qc_ammo::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["yield_pct"].get<double>(),
                80.0, 1e-6);
}
